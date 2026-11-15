"""
Bridge sensor palsu untuk simulasi OP3 (Webots).

Robot asli punya firmware OpenCR custom tim AROC yang publish:
  - /robotis/open_cr/imu   (sensor_msgs/Imu)
  - /robotis/open_cr/yaw   (std_msgs/Float64, derajat)

Di simulasi, open_cr_module bawaan ROBOTIS publish /robotis/open_cr/imu
berisi nol semua (karena ia coba buka port serial asli dan gagal), dan
/robotis/open_cr/yaw tidak ada sama sekali di source resmi.

Node ini ambil IMU ASLI dari Webots (/robotis_op3/imu, lewat
op3_extern_controller) lalu:
  1. Relay ulang isinya ke /robotis/open_cr/imu (menggantikan yang nol).
     op3_manager HARUS dijalankan dengan remap
     /robotis/open_cr/imu -> /robotis/open_cr/imu_raw supaya tidak
     rebutan publisher di topic yang sama (lihat bringup.launch.py).
  2. Hitung yaw (derajat, konvensi ENU: 0 saat identity, mengikuti
     orientation quaternion) lalu publish ke /robotis/open_cr/yaw.

Konvensi yaw firmware asli tim belum diketahui pasti (sumbernya ada di
~/RobotisSoccerYawNew, tidak tersedia saat bridge ini ditulis). Kalau
nanti arah putarnya kebalik dibanding robot asli, balik tanda di
`_yaw_deg_from_quat` atau pakai parameter `yaw_sign`.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def _yaw_deg_from_quat(x, y, z, w):
    """Yaw (derajat) dari quaternion, rotasi sekitar sumbu Z (ENU: CCW positif)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class ImuYawBridge(Node):

    def __init__(self):
        super().__init__('aroc_sim_imu_yaw_bridge')

        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)

        self.imu_pub = self.create_publisher(Imu, '/robotis/open_cr/imu', 10)
        self.yaw_pub = self.create_publisher(Float64, '/robotis/open_cr/yaw', 10)

        self.create_subscription(
            Imu, '/robotis_op3/imu', self._imu_callback, 10
        )

        self.get_logger().info(
            'aroc_sim_imu_yaw_bridge READY '
            '(/robotis_op3/imu -> /robotis/open_cr/imu + /robotis/open_cr/yaw)'
        )

    def _imu_callback(self, msg: Imu):
        # 1. Relay IMU asli ke topic yang biasanya diisi nol oleh open_cr_module.
        self.imu_pub.publish(msg)

        # 2. Hitung dan publish yaw (derajat), sesuai yang diharapkan
        #    main_state.py (current_yaw_callback mengisi self.current_yaw
        #    langsung dari Float64.data, diperlakukan sebagai derajat).
        q = msg.orientation
        yaw_deg = self.yaw_sign * _yaw_deg_from_quat(q.x, q.y, q.z, q.w)
        yaw_deg += self.yaw_offset_deg

        out = Float64()
        out.data = yaw_deg
        self.yaw_pub.publish(out)


def main():
    rclpy.init()
    node = ImuYawBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
