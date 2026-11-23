"""
Ground-truth bola dari supervisor Webots, dalam format yang dipakai GoNasional.

Menggantikan pipeline vision (YOLO -> trigonocak) di simulasi supaya behavior
bisa dites dengan bola yang benar-benar bergerak:

  - /obj_detect          (std_msgs/String  "cx,cy")  pixel di frame 450x337,
                          sama dengan frame yang dipakai main_vision + HeadControl
  - /vision/hasil_servo  (std_msgs/Float32)          jarak horizontal robot-bola (cm),
                          0.0 sekali saat bola hilang > ball_timeout
  - /vision/bola_posisi  (std_msgs/String)           "kiri" / "tengah" / "kanan"
                          relatif badan robot
  - /aroc_sim/ball_position (geometry_msgs/PointStamped) posisi bola di frame lapangan
                          (selalu, terlihat atau tidak) untuk debug dan evaluasi
  - /aroc_sim/robot_pose (geometry_msgs/PoseStamped) pose robot ground-truth di frame
                          lapangan (yaw saja), untuk evaluasi lokalisasi
  - /aroc_sim/ball_pixel (geometry_msgs/PointStamped) piksel bola ground-truth di frame
                          450x337 (x=cx, y=cy, z=jarak cm), saat bola di dalam FOV,
                          untuk mengevaluasi detektor YOLO (vision_eval)

Parameter publish_obj_detect=false mematikan /obj_detect supaya YOLO (main_vision)
yang mengisinya; jarak dan sisi bola tetap dari ground truth.

Kamera robot asli (C920) 1280x720 (16:9) dimampatkan main_vision ke 450x337 (4:3).
Proyeksi di sini meniru itu: piksel dihitung di resolusi kamera Webots
(cam_width x cam_height, piksel persegi), lalu diskalakan ke 450x337.

Subscribe (untuk skenario tes, lihat juga CLI place_ball):
  - /aroc_sim/place_ball          (geometry_msgs/Point) pindahkan bola ke x,y lapangan
  - /aroc_sim/place_ball_relative (geometry_msgs/Point) pindahkan bola ke x maju, y kiri
  - /aroc_sim/place_robot         (geometry_msgs/Pose2D) pindahkan robot ke x,y,theta lapangan
                                  (tinggi badan tetap; untuk tes lokalisasi dan "kidnap")
                                  relatif badan robot saat ini

Posisi kamera dibaca dari node CAM_REF di headSlot OP3 (lihat
worlds/krsbi_field.wbt.in), jadi proyeksi ikut gerak pan/tilt kepala.
Oklusi tidak dihitung: bola dianggap terlihat selama ada di dalam FOV.

Jangan jalankan trigonocak bersamaan kalau ada yang publish /vision/bola_cx_cy,
karena keduanya menulis ke /vision/hasil_servo.
"""

import math
import time

import rclpy
from controller import Supervisor
from geometry_msgs.msg import Point, PointStamped, Pose2D, PoseStamped
from std_msgs.msg import Float32, String

IMG_WIDTH = 450
IMG_HEIGHT = 337
CAMERA_FOV = 1.3613  # rad, fieldOfView (horizontal) Camera di RobotisOp3.proto


def _axis_angle(m):
    """Matriks rotasi 3x3 -> [ax, ay, az, sudut] untuk field rotation Webots."""
    angle = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    if math.pi - angle < 1e-6:
        # 180 derajat: sumbu dari diagonal.
        ax = [math.sqrt(max(0.0, (m[i][i] + 1.0) / 2.0)) for i in range(3)]
        k = max(range(3), key=lambda i: ax[i])
        for i in range(3):
            if i != k and m[k][i] < 0.0:
                ax[i] = -ax[i]
        return ax + [angle]
    s = 2.0 * math.sin(angle)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, angle]


def _to_local(rot, vec):
    """Vektor world -> frame lokal; rot = orientation 3x3 row-major (lokal -> world)."""
    return [
        rot[0] * vec[0] + rot[3] * vec[1] + rot[6] * vec[2],
        rot[1] * vec[0] + rot[4] * vec[1] + rot[7] * vec[2],
        rot[2] * vec[0] + rot[5] * vec[1] + rot[8] * vec[2],
    ]


class BallTracker:

    def __init__(self, supervisor, node):
        self.sup = supervisor
        self.node = node

        node.declare_parameter('publish_period', 0.066)
        node.declare_parameter('max_range', 5.0)
        node.declare_parameter('side_threshold', 0.02)
        node.declare_parameter('ball_timeout', 1.0)
        node.declare_parameter('cam_width', 640)
        node.declare_parameter('cam_height', 360)
        node.declare_parameter('publish_obj_detect', True)
        # DEF name of the robot this tracker serves (team world: OP3_1, OP3_2, ...).
        node.declare_parameter('robot_def', 'OP3')
        self.publish_period = node.get_parameter('publish_period').value
        self.max_range = node.get_parameter('max_range').value
        self.side_threshold = node.get_parameter('side_threshold').value
        self.ball_timeout = node.get_parameter('ball_timeout').value
        self.cam_width = node.get_parameter('cam_width').value
        self.cam_height = node.get_parameter('cam_height').value
        self.publish_obj_detect = node.get_parameter('publish_obj_detect').value

        self.ball = supervisor.getFromDef('BALL')
        robot_def = node.get_parameter('robot_def').value
        self.op3 = supervisor.getFromDef(robot_def)
        if self.ball is None or self.op3 is None:
            raise RuntimeError(f'World harus punya DEF BALL dan DEF {robot_def}')
        self.cam = self.op3.getField('headSlot').getMFNode(0)

        # Fokus dalam piksel kamera Webots (piksel persegi), lalu skala ke 450x337
        self.focal = (self.cam_width / 2.0) / math.tan(CAMERA_FOV / 2.0)
        self.scale_x = IMG_WIDTH / self.cam_width
        self.scale_y = IMG_HEIGHT / self.cam_height

        self.obj_pub = node.create_publisher(String, '/obj_detect', 10)
        self.dist_pub = node.create_publisher(Float32, '/vision/hasil_servo', 10)
        self.side_pub = node.create_publisher(String, '/vision/bola_posisi', 10)
        self.pos_pub = node.create_publisher(PointStamped, '/aroc_sim/ball_position', 10)
        self.pose_pub = node.create_publisher(PoseStamped, '/aroc_sim/robot_pose', 10)
        self.pixel_pub = node.create_publisher(PointStamped, '/aroc_sim/ball_pixel', 10)

        self.ball_z = self.ball.getPosition()[2]
        self.pending_place = None
        node.create_subscription(Point, '/aroc_sim/place_ball', self._on_place_field, 10)
        node.create_subscription(
            Point, '/aroc_sim/place_ball_relative', self._on_place_relative, 10)
        self.pending_robot = None
        node.create_subscription(Pose2D, '/aroc_sim/place_robot', self._on_place_robot, 10)

        self.last_seen = None
        self.lost_reported = True
        self.next_publish = 0.0
        self._rtf_mark = None  # (waktu sim, waktu nyata) untuk real-time factor

    def _on_place_field(self, msg):
        self.pending_place = ('field', msg.x, msg.y)

    def _on_place_relative(self, msg):
        self.pending_place = ('relative', msg.x, msg.y)

    def _on_place_robot(self, msg):
        self.pending_robot = (msg.x, msg.y, msg.theta)

    def _apply_place_robot(self):
        x, y, theta = self.pending_robot
        self.pending_robot = None
        z = self.op3.getPosition()[2]
        # Putar hanya yaw; roll/pitch badan saat berdiri dipertahankan. Rotasi yang
        # dipaksa tegak sempurna membuat telapak kaki miring dan robot jatuh.
        rot = self.op3.getOrientation()          # matriks 3x3 baris-mayor
        dyaw = theta - math.atan2(rot[3], rot[0])
        c, s_ = math.cos(dyaw), math.sin(dyaw)
        new = [[c * rot[0 + j] - s_ * rot[3 + j] for j in range(3)],
               [s_ * rot[0 + j] + c * rot[3 + j] for j in range(3)],
               [rot[6 + j] for j in range(3)]]
        self.op3.getField('translation').setSFVec3f([x, y, z])
        self.op3.getField('rotation').setSFRotation(_axis_angle(new))
        self.op3.resetPhysics()
        self.node.get_logger().info(
            f'Robot dipindah ke x={x:.2f} y={y:.2f} theta={math.degrees(theta):.0f} deg')

    def _apply_place(self):
        frame, x, y = self.pending_place
        self.pending_place = None
        if frame == 'relative':
            robot_pos = self.op3.getPosition()
            rot = self.op3.getOrientation()
            yaw = math.atan2(rot[3], rot[0])
            x, y = (robot_pos[0] + x * math.cos(yaw) - y * math.sin(yaw),
                    robot_pos[1] + x * math.sin(yaw) + y * math.cos(yaw))
        self.ball.getField('translation').setSFVec3f([x, y, self.ball_z])
        self.ball.resetPhysics()
        self.last_seen = None
        self.lost_reported = True
        self.node.get_logger().info(f'Bola dipindah ke lapangan x={x:.3f} y={y:.3f}')

    def _report_rtf(self, sim_now):
        """Log kecepatan simulasi tiap 10 s waktu nyata. Di bawah ~0.7x, op3_manager
        (jalan dengan jam nyata) dan Webots tidak sinkron: gerakan terpotong, tendangan
        tidak sampai, robot bisa jatuh."""
        wall = time.monotonic()
        if self._rtf_mark is None:
            self._rtf_mark = (sim_now, wall)
            return
        sim0, wall0 = self._rtf_mark
        if wall - wall0 < 10.0:
            return
        rtf = (sim_now - sim0) / (wall - wall0)
        self._rtf_mark = (sim_now, wall)
        # rclpy melarang satu baris log berganti level, jadi dua pemanggilan terpisah
        if rtf < 0.7:
            self.node.get_logger().warning(
                f'Kecepatan simulasi {rtf:.2f}x waktu nyata (terlalu lambat)')
        else:
            self.node.get_logger().info(f'Kecepatan simulasi {rtf:.2f}x waktu nyata')

    def _project(self, ball_pos):
        cam_pos = self.cam.getPosition()
        rel = [ball_pos[i] - cam_pos[i] for i in range(3)]
        x, y, z = _to_local(self.cam.getOrientation(), rel)
        if x <= 0.05 or math.sqrt(x * x + y * y + z * z) > self.max_range:
            return None
        cx = (self.cam_width / 2.0 - self.focal * y / x) * self.scale_x
        cy = (self.cam_height / 2.0 - self.focal * z / x) * self.scale_y
        if not (0 <= cx < IMG_WIDTH and 0 <= cy < IMG_HEIGHT):
            return None
        return cx, cy

    def update(self):
        if self.pending_place is not None:
            self._apply_place()
        if self.pending_robot is not None:
            self._apply_place_robot()

        now = self.sup.getTime()
        self._report_rtf(now)
        if now < self.next_publish:
            return
        self.next_publish = now + self.publish_period

        ball_pos = self.ball.getPosition()
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.node.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'field'
        pos_msg.point.x, pos_msg.point.y, pos_msg.point.z = ball_pos
        self.pos_pub.publish(pos_msg)

        # Pose robot ground-truth: untuk evaluasi lokalisasi dan untuk
        # simulasi detektor landmark (GoNasional sim_landmarks).
        robot_pos = self.op3.getPosition()
        rot = self.op3.getOrientation()
        yaw = math.atan2(rot[3], rot[0])
        pose_msg = PoseStamped()
        pose_msg.header = pos_msg.header
        pose_msg.pose.position.x = robot_pos[0]
        pose_msg.pose.position.y = robot_pos[1]
        pose_msg.pose.position.z = robot_pos[2]
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        pixel = self._project(ball_pos)

        if pixel is None:
            if not self.lost_reported and (
                    self.last_seen is None or now - self.last_seen > self.ball_timeout):
                self.dist_pub.publish(Float32(data=0.0))
                self.lost_reported = True
            return

        self.last_seen = now
        self.lost_reported = False

        robot_pos = self.op3.getPosition()
        rel = [ball_pos[i] - robot_pos[i] for i in range(3)]
        dist_cm = math.hypot(rel[0], rel[1]) * 100.0
        lateral = _to_local(self.op3.getOrientation(), rel)[1]
        if lateral > self.side_threshold:
            side = 'kiri'
        elif lateral < -self.side_threshold:
            side = 'kanan'
        else:
            side = 'tengah'

        pixel_msg = PointStamped()
        pixel_msg.header = pos_msg.header
        pixel_msg.header.frame_id = 'image_450x337'
        pixel_msg.point.x, pixel_msg.point.y = pixel
        pixel_msg.point.z = dist_cm
        self.pixel_pub.publish(pixel_msg)

        if self.publish_obj_detect:
            self.obj_pub.publish(
                String(data=f'{int(round(pixel[0]))},{int(round(pixel[1]))}'))
        self.dist_pub.publish(Float32(data=float(dist_cm)))
        self.side_pub.publish(String(data=side))


def main():
    rclpy.init()
    node = rclpy.create_node('aroc_sim_ball_tracker')
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    tracker = BallTracker(supervisor, node)
    node.get_logger().info('aroc_sim_ball_tracker READY (ground-truth /obj_detect, /vision/hasil_servo)')
    try:
        while rclpy.ok() and supervisor.step(timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            tracker.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
