"""
Evaluasi detektor bola (GoNasional main_vision / YOLO) terhadap ground truth Webots.

Butuh:
    ros2 launch aroc_sim_bridge bringup.launch.py vision:=yolo
    ros2 run GoNasional vision --ros-args -r /usb_cam_node/image_raw:=/robotis_op3/camera/image_raw \\
        -p model_path:=<model_openvino_dir> -p show_window:=false
Robot harus diam (jangan jalankan behavior). Kepala diarahkan ke pan 0 dan
--tilt, lalu bola ditaruh di tiap kombinasi --distances x --bearings.

Per posisi, deteksi /obj_detect dicocokkan ke piksel ground-truth terbaru
(/aroc_sim/ball_pixel, frame 450x337):
  hit  = deteksi dalam --tol piksel dari ground truth
  miss = deteksi yang jauh dari bola (salah objek)
Posisi dihitung "terdeteksi" kalau ada minimal satu hit.

Contoh:
    ros2 run aroc_sim_bridge vision_eval --distances 0.5 1 2 3 4 --bearings -20 0 20
"""

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class VisionEval:

    def __init__(self, node, tol):
        self.node = node
        self.tol = tol
        self.gt = None          # (waktu, cx, cy, jarak_cm)
        self.samples = []       # (cx, cy) deteksi YOLO selama jendela pengukuran
        self.recording = False

        node.create_subscription(PointStamped, '/aroc_sim/ball_pixel', self._on_gt, 10)
        node.create_subscription(String, '/obj_detect', self._on_detect, 10)
        self.place_pub = node.create_publisher(Point, '/aroc_sim/place_ball_relative', 10)
        self.module_pub = node.create_publisher(String, '/robotis/enable_ctrl_module', 10)
        self.head_pub = node.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', 10)

    def _on_gt(self, msg):
        self.gt = (time.monotonic(), msg.point.x, msg.point.y, msg.point.z)

    def _on_detect(self, msg):
        if not self.recording:
            return
        try:
            cx, cy = (float(v) for v in msg.data.strip().split('\n')[0].split(','))
        except ValueError:
            return
        self.samples.append((cx, cy, self.gt))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def point_head(self, pan, tilt):
        self.module_pub.publish(String(data='head_control_module'))
        self.spin_for(0.5)
        msg = JointState()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]
        self.head_pub.publish(msg)
        self.spin_for(1.5)

    def measure(self, distance, bearing_deg, settle, window):
        rad = math.radians(bearing_deg)
        self.place_pub.publish(Point(x=distance * math.cos(rad), y=distance * math.sin(rad)))
        self.gt = None
        self.spin_for(settle)

        self.samples = []
        self.recording = True
        self.spin_for(window)
        self.recording = False

        gt = self.gt
        if gt is None or time.monotonic() - gt[0] > 0.5:
            return {'in_fov': False}
        errors, misses = [], 0
        for cx, cy, gt_at in self.samples:
            ref = gt_at or gt
            err = math.hypot(cx - ref[1], cy - ref[2])
            if err <= self.tol:
                errors.append(err)
            else:
                misses += 1
        return {
            'in_fov': True,
            'gt': (gt[1], gt[2]),
            'dist_cm': gt[3],
            'hits': len(errors),
            'misses': misses,
            'rate_hz': len(errors) / window,
            'err_px': statistics.median(errors) if errors else None,
        }


def main():
    parser = argparse.ArgumentParser(description='Evaluasi detektor bola vs ground truth Webots.')
    parser.add_argument('--distances', type=float, nargs='+', default=[0.5, 1.0, 1.5, 2.0, 3.0, 4.0])
    parser.add_argument('--bearings', type=float, nargs='+', default=[-20.0, 0.0, 20.0])
    parser.add_argument('--tilt', type=float, default=-0.4, help='tilt kepala (rad, negatif = menunduk)')
    parser.add_argument('--settle', type=float, default=1.5, help='jeda setelah bola dipindah (s)')
    parser.add_argument('--window', type=float, default=3.0, help='lama pengukuran per posisi (s)')
    parser.add_argument('--tol', type=float, default=25.0, help='batas jarak hit (piksel 450x337)')
    parser.add_argument('--label', default='', help='nama model untuk baris ringkasan')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('aroc_sim_vision_eval')
    ev = VisionEval(node, args.tol)

    deadline = time.monotonic() + 10.0
    while ev.place_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if ev.place_pub.get_subscription_count() == 0:
        node.get_logger().error('ball_tracker tidak ditemukan (bringup world:=field vision:=yolo)')
        raise SystemExit(1)

    ev.point_head(0.0, args.tilt)
    print(f"{'jarak':>6} {'arah':>6} {'gt cx,cy':>11} {'hit':>4} {'salah':>5} "
          f"{'Hz':>5} {'err px':>6}  hasil")
    detected = total = 0
    hits_total = misses_total = 0
    errors = []
    for distance in args.distances:
        for bearing in args.bearings:
            r = ev.measure(distance, bearing, args.settle, args.window)
            if not r['in_fov']:
                print(f'{distance:6.2f} {bearing:6.1f} {"-":>11} {"":>4} {"":>5} {"":>5} '
                      f'{"":>6}  di luar FOV')
                continue
            total += 1
            detected += r['hits'] > 0
            hits_total += r['hits']
            misses_total += r['misses']
            if r['err_px'] is not None:
                errors.append(r['err_px'])
            err = f"{r['err_px']:.1f}" if r['err_px'] is not None else '-'
            print(f"{distance:6.2f} {bearing:6.1f} {r['gt'][0]:5.0f},{r['gt'][1]:<5.0f} "
                  f"{r['hits']:4d} {r['misses']:5d} {r['rate_hz']:5.1f} {err:>6}  "
                  f"{'TERDETEKSI' if r['hits'] else 'tidak terdeteksi'}")

    median_err = f'{statistics.median(errors):.1f}' if errors else '-'
    label = f'[{args.label}] ' if args.label else ''
    print(f'\n{label}posisi terdeteksi {detected}/{total}, '
          f'deteksi benar {hits_total}, deteksi salah {misses_total}, '
          f'median error {median_err} px')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
