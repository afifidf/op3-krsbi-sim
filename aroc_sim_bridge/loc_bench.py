"""Benchmark lokalisasi GoNasional dengan landmark dari kamera, robot diam.

Robot dipindah (supervisor) ke beberapa pose, kepala scan, lalu error estimasi
/world_model dibandingkan dengan pose sebenarnya. Tidak butuh robot berjalan,
jadi tetap valid walau simulasi lambat (YOLO + Webots di laptop).

Mode:
  offset  pose awal estimasi digeser --offset m dari pose sebenarnya (/initialpose);
          mengukur apakah filter menarik estimasi ke pose benar.
  kidnap  robot dipindah tanpa memberi tahu lokalisasi; mengukur apakah filter
          menyatakan LOST lalu pulih, dan berapa lama.

Jalankan setelah:
  ros2 launch aroc_sim_bridge bringup.launch.py gui:=false vision:=yolo model:=<folder>
  ros2 launch GoNasional aroc_soccer.launch.py sim:=true referee:=false landmarks:=vision behavior:=false
lalu:
  ros2 run aroc_sim_bridge loc_bench --mode offset
"""

import argparse
import math
import statistics
import time

import rclpy
from aroc_interfaces.msg import LandmarkArray, WorldModel
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

POSES = [(-2.5, 0.0, 0.0), (-1.0, 1.5, -0.6), (1.0, -1.2, 0.4),
         (2.5, 0.5, 0.0), (-3.0, -2.0, 0.8), (0.5, 2.0, -1.2)]
STATE_NAMES = ('TRACKING', 'UNCERTAIN', 'LOST')


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bench:

    def __init__(self, node):
        self.node = node
        self.gt = None
        self.wm = None
        self.landmarks = {}
        self.place_pub = node.create_publisher(Pose2D, '/aroc_sim/place_robot', 10)
        self.init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.head_pub = node.create_publisher(String, '/head/mode', 10)
        self.cmd_pub = node.create_publisher(String, '/motion/command', 10)
        self.motion_status = None
        node.create_subscription(String, '/motion/status', self._on_status, 10)
        node.create_subscription(PoseStamped, '/aroc_sim/robot_pose', self._on_gt, 10)
        node.create_subscription(WorldModel, '/world_model', self._on_wm, 10)
        node.create_subscription(LandmarkArray, '/perception/landmarks', self._on_lm, 10)

    def _on_status(self, msg):
        self.motion_status = msg.data

    def stand(self, timeout=40.0):
        end = time.monotonic() + timeout
        while self.motion_status != 'READY' and time.monotonic() < end:
            self.cmd_pub.publish(String(data='stand'))
            self.spin_for(1.0)
        return self.motion_status == 'READY'

    def _on_gt(self, msg):
        self.gt = (msg.pose.position.x, msg.pose.position.y, _yaw(msg.pose.orientation))

    def _on_wm(self, msg):
        self.wm = msg

    def _on_lm(self, msg):
        for lm in msg.landmarks:
            self.landmarks[lm.type] = self.landmarks.get(lm.type, 0) + 1

    def spin_for(self, seconds, callback=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if callback:
                callback()

    def error(self):
        if self.gt is None or self.wm is None:
            return None
        p = self.wm.pose
        return (math.hypot(p.x - self.gt[0], p.y - self.gt[1]),
                abs((p.theta - self.gt[2] + math.pi) % (2 * math.pi) - math.pi))

    def set_estimate(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'field'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        self.init_pub.publish(msg)

    def run_pose(self, pose, mode, offset, dwell):
        self.place_pub.publish(Pose2D(x=pose[0], y=pose[1], theta=pose[2]))
        self.spin_for(2.0)
        self.landmarks = {}
        if mode == 'offset':
            ang = math.atan2(-pose[1], -pose[0]) if any(pose[:2]) else 0.0
            self.set_estimate(pose[0] + offset * math.cos(ang), pose[1] + offset * math.sin(ang),
                              pose[2])
        start = time.monotonic()
        samples, states, recovered = [], set(), [None]

        def sample():
            err = self.error()
            if err is None:
                return
            t = time.monotonic() - start
            states.add(self.wm.localization_state)
            if recovered[0] is None and err[0] < 0.3 and self.wm.localization_state == 0:
                recovered[0] = t
            if t > dwell - 10.0:
                samples.append(err)

        while time.monotonic() - start < dwell:
            self.head_pub.publish(String(data='scan'))
            self.spin_for(1.0, sample)
        return samples, states, recovered[0], dict(self.landmarks)


def main():
    parser = argparse.ArgumentParser(description='Benchmark lokalisasi dengan landmark kamera.')
    parser.add_argument('--mode', choices=['offset', 'kidnap'], default='offset')
    parser.add_argument('--offset', type=float, default=0.6, help='geseran estimasi awal (m)')
    parser.add_argument('--dwell', type=float, default=40.0, help='lama per pose (s)')
    parser.add_argument('--poses', type=int, default=len(POSES), help='jumlah pose dipakai')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('aroc_sim_loc_bench')
    bench = Bench(node)
    deadline = time.monotonic() + 20.0
    while (bench.gt is None or bench.wm is None) and time.monotonic() < deadline:
        bench.spin_for(0.5)
    if bench.gt is None or bench.wm is None:
        node.get_logger().error('butuh /aroc_sim/robot_pose (bringup) dan /world_model (aroc_soccer)')
        raise SystemExit(1)

    if not bench.stand():
        node.get_logger().error(f'motion_manager tidak READY (status {bench.motion_status})')
        raise SystemExit(1)
    print(f"{'pose':>20} {'err med':>8} {'err max':>8} {'yaw':>5} {'pulih s':>8}  state  landmark")
    all_err = []
    for pose in POSES[:args.poses]:
        samples, states, recovered, lms = bench.run_pose(pose, args.mode, args.offset, args.dwell)
        if not samples:
            print(f'{str(pose):>20}  tidak ada data')
            continue
        errs = [e for e, _ in samples]
        all_err += errs
        yaw = math.degrees(statistics.median(t for _, t in samples))
        rec = f'{recovered:.0f}' if recovered is not None else '-'
        names = '/'.join(STATE_NAMES[s] for s in sorted(states))
        lm = ' '.join(f'{k}:{v}' for k, v in sorted(lms.items())) or '-'
        print(f'{str(pose):>20} {statistics.median(errs):8.2f} {max(errs):8.2f} {yaw:5.0f} '
              f'{rec:>8}  {names}  {lm}', flush=True)
    if all_err:
        all_err.sort()
        print(f'\n[{args.mode}] median {statistics.median(all_err):.2f} m, '
              f'p90 {all_err[int(0.9 * len(all_err))]:.2f} m  '
              f'(landmark: 1 tiang, 2 penalti, 3 tengah, 4 sudut)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
