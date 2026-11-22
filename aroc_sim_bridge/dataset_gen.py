"""
Generator dataset YOLO untuk bola, dengan label otomatis dari ground truth Webots.

Menjalankan Webots sendiri (world dataset_field.wbt, mode fast, tanpa window) lalu
bertindak sebagai supervisor "dataset_camera". Tiap sampel mengacak:
  - pose kamera: posisi di lapangan, tinggi, yaw, tilt (menunduk), sedikit roll
  - bola: titik acak di gambar diproyeksikan ke lantai, rotasi acak
  - noise kamera, arah dan intensitas lampu matahari (DirectionalLight SUN)
  - tiap --relight-every sampel: tekstur latar dan kecerahannya (lambat: proto
    dibangun ulang dan tekstur diunduh sekali lalu di-cache Webots)
Sebagian sampel (--neg-ratio) tanpa bola di gambar, label kosong.

Gambar disimpan 450x337 (dimampatkan dari 640x360, sama seperti main_vision
memperlakukan C920 1280x720) kecuali --no-squash. Label YOLO dinormalisasi, jadi
tidak berubah oleh resize. Bounding box dihitung dari siluet bola (titik-titik
di permukaan bola diproyeksikan), dipotong di tepi gambar.

Pakai (ROS 2 sudah di-source):
    ros2 run aroc_sim_bridge dataset_gen --out ~/datasets/bola_sim --count 2000

Hasil: <out>/images/{train,val}, <out>/labels/{train,val}, <out>/data.yaml (kelas 0 = bola).
Untuk digabung ke dataset tim yang indeks kelas bolanya bukan 0 (mis. gkeep_sementara:
0 GK, 1 Robot, 2 bola), pakai --class-id 2; data.yaml tetap harus diambil dari dataset tim.
"""

import argparse
import math
import os
import random
import subprocess
import sys
import time

WORLD_NAME = 'dataset_field.wbt'
BALL_RADIUS = 0.07          # RobocupSoccerBall size 1
CAM_FOV = 1.3613            # rad, horizontal; sama dengan Camera di RobotisOp3.proto
CAM_W, CAM_H = 640, 360
OUT_W, OUT_H = 450, 337     # frame yang dipakai main_vision
TEXTURES = [
    'stadium', 'stadium_dry', 'noon_sunny_empty', 'noon_cloudy_empty',
    'noon_building_overcast', 'morning_cloudy_empty', 'dawn_cloudy_empty',
    'entrance_hall', 'empty_office', 'factory', 'music_hall',
]


def _ensure_env(port):
    """Jalankan ulang proses ini dengan env yang dibutuhkan controller Webots."""
    if os.environ.get('AROC_DATASET_ENV') == '1':
        return
    webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
    controller_lib = os.path.join(webots_home, 'lib', 'controller')
    env = dict(os.environ)
    env.update({
        'AROC_DATASET_ENV': '1',
        'WEBOTS_HOME': webots_home,
        'WEBOTS_CONTROLLER_URL': f'ipc://{port}/dataset_camera',
        'LD_LIBRARY_PATH': controller_lib + os.pathsep + env.get('LD_LIBRARY_PATH', ''),
        # NumPy 1.x sistem didahulukan (cv2 sistem dikompilasi untuknya).
        'PYTHONPATH': os.pathsep.join([
            '/usr/lib/python3/dist-packages',
            os.path.join(controller_lib, 'python'),
            env.get('PYTHONPATH', '')]),
    })
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


def _rot_matrix(yaw, pitch, roll):
    """R = Rz(yaw) Ry(pitch) Rx(roll); pitch positif = menunduk (sumbu x ke bawah)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_angle(m):
    angle = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    s = 2.0 * math.sin(angle)
    axis = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    norm = math.sqrt(sum(a * a for a in axis)) or 1.0
    return [a / norm for a in axis] + [angle]


def _sphere_points(n=200):
    """Titik Fibonacci di permukaan bola satuan, untuk menghitung siluet."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    pts = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        r = math.sqrt(1.0 - z * z)
        pts.append((r * math.cos(golden * i), r * math.sin(golden * i), z))
    return pts


class DatasetGen:

    def __init__(self, sup, rng, args):
        self.sup = sup
        self.rng = rng
        self.args = args
        self.timestep = int(sup.getBasicTimeStep())
        self.camera = sup.getDevice('camera')
        self.camera.enable(self.timestep)
        self.cambot = sup.getSelf()
        self.ball = sup.getFromDef('BALL')
        self.cam_node = sup.getFromDef('CAM')
        self.sun = sup.getFromDef('SUN')
        self.focal = (CAM_W / 2.0) / math.tan(CAM_FOV / 2.0)
        self.sphere = _sphere_points()

    # --- acak scene -------------------------------------------------------

    def relight(self):
        texture = self.rng.choice(TEXTURES)
        luminosity = self.rng.uniform(0.6, 1.6)
        for name in ('BG', 'LIGHT'):
            node = self.sup.getFromDef(name)
            node.getField('texture').setSFString(texture)
            node = self.sup.getFromDef(name)  # proto dibangun ulang setelah field berubah
            node.getField('luminosity').setSFFloat(luminosity)
        self.sup.step(self.timestep)
        self.ball = self.sup.getFromDef('BALL')

    def place_camera(self):
        a = self.args
        pos = [self.rng.uniform(-4.5, 4.5), self.rng.uniform(-3.0, 3.0),
               self.rng.uniform(a.cam_height_min, a.cam_height_max)]
        rot = _rot_matrix(self.rng.uniform(-math.pi, math.pi),
                          self.rng.uniform(a.tilt_min, a.tilt_max),
                          self.rng.gauss(0.0, 0.04))
        self.cambot.getField('translation').setSFVec3f(pos)
        self.cambot.getField('rotation').setSFRotation(_axis_angle(rot))
        self.cam_node.getField('noise').setSFFloat(self.rng.uniform(0.0, a.noise_max))
        azimuth = self.rng.uniform(-math.pi, math.pi)
        elevation = self.rng.uniform(0.3, 1.4)
        self.sun.getField('direction').setSFVec3f([
            -math.cos(elevation) * math.cos(azimuth),
            -math.cos(elevation) * math.sin(azimuth),
            -math.sin(elevation)])
        self.sun.getField('intensity').setSFFloat(self.rng.uniform(0.2, 1.4))
        return pos, rot

    def _ray_to_ground(self, pos, rot, u, v):
        d = (1.0, (CAM_W / 2.0 - u) / self.focal, (CAM_H / 2.0 - v) / self.focal)
        w = [sum(rot[i][j] * d[j] for j in range(3)) for i in range(3)]
        if w[2] >= -1e-6:
            return None
        t = (BALL_RADIUS - pos[2]) / w[2]
        ball = [pos[0] + t * w[0], pos[1] + t * w[1], BALL_RADIUS]
        horiz = math.hypot(ball[0] - pos[0], ball[1] - pos[1])
        if not (self.args.min_range <= horiz <= self.args.max_range):
            return None
        if abs(ball[0]) > 5.0 or abs(ball[1]) > 3.6:
            return None
        return ball

    def place_ball(self, pos, rot, visible):
        if visible:
            for _ in range(50):
                ball = self._ray_to_ground(
                    pos, rot, self.rng.uniform(0, CAM_W), self.rng.uniform(0, CAM_H))
                if ball:
                    break
            else:
                return None
        else:
            # Di belakang arah pandang kamera (proyeksi sumbu x kamera ke lantai)
            fx, fy = rot[0][0], rot[1][0]
            norm = math.hypot(fx, fy) or 1.0
            dist = self.rng.uniform(0.5, 2.0)
            ball = [pos[0] - dist * fx / norm, pos[1] - dist * fy / norm, BALL_RADIUS]
        axis = [self.rng.gauss(0, 1) for _ in range(3)]
        norm = math.sqrt(sum(a * a for a in axis)) or 1.0
        self.ball.getField('translation').setSFVec3f(ball)
        self.ball.getField('rotation').setSFRotation(
            [a / norm for a in axis] + [self.rng.uniform(0, 2 * math.pi)])
        self.ball.resetPhysics()
        return ball

    # --- label ------------------------------------------------------------

    def bbox(self, cam_pos, rot):
        """Bounding box ternormalisasi (cx, cy, w, h) atau None kalau tidak layak dilabel."""
        ball = self.ball.getPosition()
        rel = [ball[i] - cam_pos[i] for i in range(3)]
        if sum(rot[j][0] * rel[j] for j in range(3)) < -BALL_RADIUS:
            return 'outside'  # seluruh bola di belakang kamera
        us, vs = [], []
        for p in self.sphere:
            world = [rel[i] + BALL_RADIUS * p[i] for i in range(3)]
            x, y, z = (sum(rot[j][i] * world[j] for j in range(3)) for i in range(3))
            if x <= 0.02:
                return None
            us.append(CAM_W / 2.0 - self.focal * y / x)
            vs.append(CAM_H / 2.0 - self.focal * z / x)
        x1, x2, y1, y2 = min(us), max(us), min(vs), max(vs)
        full = (x2 - x1) * (y2 - y1)
        cx1, cx2 = max(0.0, x1), min(float(CAM_W), x2)
        cy1, cy2 = max(0.0, y1), min(float(CAM_H), y2)
        if cx2 <= cx1 or cy2 <= cy1:
            return 'outside'
        if (cx2 - cx1) * (cy2 - cy1) < self.args.min_visible * full:
            return None
        w, h = (cx2 - cx1) / CAM_W, (cy2 - cy1) / CAM_H
        if min(w * OUT_W, h * OUT_H) < self.args.min_px:
            return None
        return ((cx1 + cx2) / 2.0 / CAM_W, (cy1 + cy2) / 2.0 / CAM_H, w, h)

    def grab(self):
        import cv2
        import numpy as np
        raw = self.camera.getImage()
        img = np.frombuffer(raw, np.uint8).reshape(CAM_H, CAM_W, 4)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if self.args.squash:
            img = cv2.resize(img, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
        return img


def _write_data_yaml(out, class_id):
    names = ''.join(f'  {i}: {"bola" if i == class_id else f"kelas_{i}"}\n'
                    for i in range(class_id + 1))
    with open(os.path.join(out, 'data.yaml'), 'w') as f:
        f.write(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n{names}')


def main():
    parser = argparse.ArgumentParser(description='Generator dataset bola YOLO dari Webots.')
    parser.add_argument('--out', required=True, help='folder output dataset')
    parser.add_argument('--count', type=int, default=1000, help='jumlah gambar')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--val-ratio', type=float, default=0.1)
    parser.add_argument('--neg-ratio', type=float, default=0.1, help='porsi gambar tanpa bola')
    parser.add_argument('--relight-every', type=int, default=500)
    parser.add_argument('--cam-height-min', type=float, default=0.40)
    parser.add_argument('--cam-height-max', type=float, default=0.50)
    parser.add_argument('--tilt-min', type=float, default=0.0, help='rad, 0 = lurus ke depan')
    parser.add_argument('--tilt-max', type=float, default=1.2, help='rad, menunduk')
    parser.add_argument('--min-range', type=float, default=0.2)
    parser.add_argument('--max-range', type=float, default=6.0)
    parser.add_argument('--min-visible', type=float, default=0.4,
                        help='bola terpotong tepi gambar di bawah porsi ini dilewati')
    parser.add_argument('--min-px', type=float, default=4.0,
                        help='bola lebih kecil dari ini (piksel 450x337) dilewati')
    parser.add_argument('--noise-max', type=float, default=0.03)
    parser.add_argument('--no-squash', dest='squash', action='store_false',
                        help='simpan 640x360 tanpa dimampatkan ke 450x337')
    parser.add_argument('--class-id', type=int, default=0,
                        help='indeks kelas bola di label (sesuaikan dengan dataset tim)')
    parser.add_argument('--port', type=int, default=1235)
    parser.add_argument('--no-launch', action='store_true',
                        help='jangan jalankan Webots (sudah dibuka manual di --port)')
    args = parser.parse_args()
    _ensure_env(args.port)

    out = os.path.abspath(os.path.expanduser(args.out))
    for split in ('train', 'val'):
        os.makedirs(os.path.join(out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(out, 'labels', split), exist_ok=True)

    webots = None
    if not args.no_launch:
        from ament_index_python.packages import get_package_share_directory
        world = os.path.join(get_package_share_directory('aroc_sim_bridge'), 'worlds', WORLD_NAME)
        log = open(os.path.join(out, 'webots.log'), 'w')
        webots = subprocess.Popen(
            [os.path.join(os.environ['WEBOTS_HOME'], 'webots'), '--batch', '--mode=fast',
             '--no-rendering', '--minimize', '--stdout', '--stderr',
             f'--port={args.port}', world],
            stdout=log, stderr=subprocess.STDOUT)

    import cv2
    from controller import Supervisor

    rng = random.Random(args.seed)
    sup = None
    done = False
    try:
        sup = Supervisor()
        gen = DatasetGen(sup, rng, args)
        val_every = max(1, round(1.0 / args.val_ratio)) if args.val_ratio > 0 else 0
        stats = {'bola': 0, 'kosong': 0, 'ulang': 0}
        start = time.monotonic()
        i = 0
        while i < args.count:
            if i % args.relight_every == 0:
                gen.relight()
            cam_pos, rot = gen.place_camera()
            want_ball = rng.random() >= args.neg_ratio
            if gen.place_ball(cam_pos, rot, want_ball) is None:
                stats['ulang'] += 1
                continue
            sup.step(gen.timestep)
            sup.step(gen.timestep)
            box = gen.bbox(cam_pos, rot)
            if box is None or want_ball != (box != 'outside'):
                stats['ulang'] += 1
                continue
            split = 'val' if val_every and i % val_every == 0 else 'train'
            name = f'sim_{args.seed}_{i:06d}'
            cv2.imwrite(os.path.join(out, 'images', split, name + '.jpg'), gen.grab(),
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            with open(os.path.join(out, 'labels', split, name + '.txt'), 'w') as f:
                if box != 'outside':
                    f.write('{} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(args.class_id, *box))
                    stats['bola'] += 1
                else:
                    stats['kosong'] += 1
            i += 1
            if i % 100 == 0 or i == args.count:
                rate = i / (time.monotonic() - start)
                print(f'{i}/{args.count} gambar ({rate:.1f}/s)', flush=True)
        _write_data_yaml(out, args.class_id)
        print(f"Selesai: {stats['bola']} dengan bola, {stats['kosong']} tanpa bola, "
              f"{stats['ulang']} percobaan diulang. data.yaml di {out}", flush=True)
        done = True
    finally:
        if webots and sup is not None:
            sup.simulationQuit(0)  # tutup Webots dengan rapi
        if webots:
            try:
                webots.wait(timeout=10)
            except subprocess.TimeoutExpired:
                webots.terminate()
                try:
                    webots.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    webots.kill()
    if done:
        # libController keluar dengan status 1 saat cleanup setelah Webots ditutup
        os._exit(0)


if __name__ == '__main__':
    main()
