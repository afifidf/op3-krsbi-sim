"""Pantau tiga robot team.launch.py sekaligus (tiap robot di domain ROS sendiri).

    ros2 run aroc_sim_bridge team_monitor                 # cetak tiap 5 s
    ros2 run aroc_sim_bridge team_monitor --duration 600 --csv team.csv

Per robot: peran, status motion, pose asli (Webots) vs estimasi lokalisasi.
Plus posisi bola asli, gol, dan ringkasan di akhir: berapa lama ada dua
striker sekaligus, berapa lama tanpa striker, dan jarak pemain ke bola.
"""

import argparse
import csv
import math
import threading
import time

import rclpy
from aroc_interfaces.msg import WorldModel
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

GOAL_X = 4.5
GOAL_HALF_WIDTH = 0.8   # RobotstadiumSoccerField


class RobotView:
    def __init__(self, player, domain):
        self.player = player
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node(f'team_monitor_{player}', context=self.context)
        self.role = '-'
        self.motion = '-'
        self.gt = None
        self.est = None
        self.loc_state = None
        self.ball = None
        n = self.node
        n.create_subscription(String, '/behavior/role', self._on_role, 10)
        n.create_subscription(String, '/motion/status', self._on_motion, 10)
        n.create_subscription(PoseStamped, '/aroc_sim/robot_pose', self._on_gt, 10)
        n.create_subscription(PointStamped, '/aroc_sim/ball_position', self._on_ball, 10)
        n.create_subscription(WorldModel, '/world_model', self._on_wm, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(n)
        self._running = True
        self._thread = threading.Thread(target=self._spin)
        self._thread.start()

    def _spin(self):
        while self._running:
            self.executor.spin_once(timeout_sec=0.1)

    def _on_role(self, msg):
        self.role = msg.data

    def _on_motion(self, msg):
        self.motion = msg.data

    def _on_gt(self, msg):
        q = msg.pose.orientation
        self.gt = (msg.pose.position.x, msg.pose.position.y,
                   2.0 * math.atan2(q.z, q.w), msg.pose.position.z)

    def _on_ball(self, msg):
        self.ball = (msg.point.x, msg.point.y)

    def _on_wm(self, msg):
        self.est = (msg.pose.x, msg.pose.y, msg.pose.theta)
        self.loc_state = msg.localization_state

    def close(self):
        self._running = False
        self._thread.join()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)


def main():
    parser = argparse.ArgumentParser(description='Pantau team play 3 OP3 di Webots.')
    parser.add_argument('--robots', type=int, default=3)
    parser.add_argument('--domain-base', type=int, default=10)
    parser.add_argument('--period', type=float, default=5.0, help='detik antar baris')
    parser.add_argument('--duration', type=float, default=0.0, help='0 = sampai Ctrl+C')
    parser.add_argument('--csv', default='')
    args = parser.parse_args()

    views = [RobotView(k, args.domain_base + k) for k in range(1, args.robots + 1)]
    writer = None
    if args.csv:
        f = open(args.csv, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(['t', 'player', 'role', 'motion', 'gt_x', 'gt_y', 'gt_z',
                         'est_x', 'est_y', 'loc_state', 'ball_x', 'ball_y'])
    start = time.monotonic()
    samples = double = none = 0
    fallen = {v.player: 0 for v in views}
    goals = 0
    in_goal = None                  # unknown until the first sample
    try:
        while not args.duration or time.monotonic() - start < args.duration:
            time.sleep(args.period)
            t = time.monotonic() - start
            ball = next((v.ball for v in views if v.ball is not None), None)
            parts = []
            strikers = 0
            for v in views:
                if v.role == 'striker' and v.motion in ('READY', 'WALKING', 'KICKING'):
                    strikers += 1
                if v.gt is not None and v.gt[3] < 0.18:
                    fallen[v.player] += 1
                gt = '-' if v.gt is None else f'({v.gt[0]:+.2f},{v.gt[1]:+.2f})'
                err = '-'
                if v.gt is not None and v.est is not None:
                    err = f'{math.hypot(v.gt[0] - v.est[0], v.gt[1] - v.est[1]):.2f}'
                dball = '-'
                if v.gt is not None and ball is not None:
                    dball = f'{math.hypot(v.gt[0] - ball[0], v.gt[1] - ball[1]):.1f}'
                parts.append(f'#{v.player} {v.role:<9} {v.motion:<10} {gt} err {err} bola {dball}')
                if writer:
                    writer.writerow([f'{t:.1f}', v.player, v.role, v.motion,
                                     *(v.gt[:3] if v.gt else ('', '', '')),
                                     *(v.est[:2] if v.est else ('', '')),
                                     v.loc_state, *(ball if ball else ('', ''))])
            samples += 1
            double += strikers > 1
            none += strikers == 0
            if ball is not None:
                now_in = ball[0] > GOAL_X and abs(ball[1]) < GOAL_HALF_WIDTH
                goals += now_in and in_goal is False
                in_goal = now_in
            b = '-' if ball is None else f'({ball[0]:+.2f},{ball[1]:+.2f})'
            print(f'[{t:6.0f}s] bola {b} | ' + ' | '.join(parts), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if samples:
            print(f'\nRingkasan {samples} sampel: dua striker {100.0 * double / samples:.0f}%, '
                  f'tanpa striker {100.0 * none / samples:.0f}%, gol {goals}, '
                  f'sampel jatuh per robot {fallen}', flush=True)
        if writer:
            f.close()
        for v in views:
            try:
                v.close()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
