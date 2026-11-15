"""
Taruh bola di simulasi untuk skenario tes (butuh ball_tracker jalan, world:=field).

Relatif robot (arah dalam derajat, CCW/kiri positif, sama dengan yaw dari bridge):
    ros2 run aroc_sim_bridge place_ball --bearing -15 --distance 1.0

Posisi lapangan (meter, gawang di x = +-4.5):
    ros2 run aroc_sim_bridge place_ball --x 1.0 --y -0.5

Untuk tes tendangan GoNasional, pakai --bearing = BALL_YAW_OFFSETS[index bola]
(main_state.py), ditaruh setelah tombol `mode` dan sebelum `start`: target yaw
dihitung dari saved_yaw saat `mode`, jadi bola harus searah heading target.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Point


def main():
    parser = argparse.ArgumentParser(description='Pindahkan bola di simulasi Webots.')
    parser.add_argument('--bearing', type=float, help='arah bola relatif robot (derajat, kiri +)')
    parser.add_argument('--distance', type=float, default=1.0, help='jarak bola dari robot (m)')
    parser.add_argument('--x', type=float, help='posisi x lapangan (m)')
    parser.add_argument('--y', type=float, help='posisi y lapangan (m)')
    args, ros_args = parser.parse_known_args()

    if args.bearing is not None:
        topic = '/aroc_sim/place_ball_relative'
        rad = math.radians(args.bearing)
        point = Point(x=args.distance * math.cos(rad), y=args.distance * math.sin(rad))
    elif args.x is not None and args.y is not None:
        topic = '/aroc_sim/place_ball'
        point = Point(x=args.x, y=args.y)
    else:
        parser.error('pakai --bearing [--distance] atau --x dan --y')

    rclpy.init(args=ros_args)
    node = rclpy.create_node('aroc_sim_place_ball')
    pub = node.create_publisher(Point, topic, 10)

    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error('ball_tracker tidak ditemukan (jalankan bringup world:=field)')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    pub.publish(point)
    # Beri waktu DDS mengirim sebelum node ditutup.
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f'{topic} <- x={point.x:.3f} y={point.y:.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
