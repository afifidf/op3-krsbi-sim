"""Bringup simulasi OP3 lengkap: Webots + op3_manager (mode simulasi) + bridge.

Pakai:
    ros2 launch aroc_sim_bridge bringup.launch.py                 # lapangan KRSBI + bola
    ros2 launch aroc_sim_bridge bringup.launch.py world:=empty    # world kosong bawaan ROBOTIS
    ros2 launch aroc_sim_bridge bringup.launch.py gui:=false
    ros2 launch aroc_sim_bridge bringup.launch.py vision:=yolo   # /obj_detect dari YOLO

Di world:=field, ball_tracker publish ground-truth /obj_detect, /vision/hasil_servo,
dan /vision/bola_posisi, jadi GoNasional cukup:
    ros2 run GoNasional state
(trigonocak dan vision tidak perlu dijalankan.)

Dengan vision:=yolo, /obj_detect tidak dikirim ball_tracker tapi oleh detektor tim
(GoNasional vision), yang ikut dijalankan kalau model:= diisi:
    ros2 launch aroc_sim_bridge bringup.launch.py vision:=yolo model:=<model_openvino_dir>
Jarak dan sisi bola tetap dari ground truth.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher

WEBOTS_PORT = '1234'

# Page tendang tim (KICK_KIRI/KICK_KANAN di GoNasional main_state.py) ada di
# motion file tim sendiri dan kosong di motion file bawaan ROBOTIS. Untuk
# simulasi, slot itu diisi salinan page tendang bawaan: {page tim: page bawaan}.
ACTION_PAGE_SIZE = 512
SIM_KICK_PAGES = {155: 84, 156: 83}  # 84 'l_kick', 83 'r_kick'


def _render_field_world(cam_width, cam_height):
    """Isi path RobotisOp3.proto dan resolusi kamera ke template, tulis ke file sementara."""
    bridge_pkg = get_package_share_directory('aroc_sim_bridge')
    op3_proto = os.path.join(
        get_package_share_directory('op3_webots_ros2'), 'protos', 'RobotisOp3.proto')
    with open(os.path.join(bridge_pkg, 'worlds', 'krsbi_field.wbt.in')) as f:
        world = (f.read().replace('__OP3_PROTO__', op3_proto)
                 .replace('__CAM_W__', str(cam_width))
                 .replace('__CAM_H__', str(cam_height)))
    out = tempfile.NamedTemporaryFile('w', suffix='_krsbi_field.wbt', delete=False)
    out.write(world)
    out.close()
    return out.name


def _build_sim_motion_file():
    """Salin motion file bawaan ROBOTIS dengan page tendang dipindah ke slot tim."""
    src = os.path.join(
        get_package_share_directory('op3_action_module'), 'data', 'motion_4095.bin')
    with open(src, 'rb') as f:
        data = bytearray(f.read())
    for dst_page, src_page in SIM_KICK_PAGES.items():
        start = src_page * ACTION_PAGE_SIZE
        data[dst_page * ACTION_PAGE_SIZE:(dst_page + 1) * ACTION_PAGE_SIZE] = \
            data[start:start + ACTION_PAGE_SIZE]
    out = tempfile.NamedTemporaryFile('wb', suffix='_motion_4095_sim.bin', delete=False)
    out.write(data)
    out.close()
    return out.name


def _model_imgsz(model_dir):
    """Ukuran input model OpenVINO dari metadata.yaml hasil export ultralytics."""
    import yaml
    with open(os.path.join(model_dir, 'metadata.yaml')) as f:
        imgsz = yaml.safe_load(f)['imgsz']
    return int(imgsz[0] if isinstance(imgsz, list) else imgsz)


def _launch_setup(context):
    gui = LaunchConfiguration('gui')
    world_name = LaunchConfiguration('world').perform(context)
    vision = LaunchConfiguration('vision').perform(context)
    model = LaunchConfiguration('model').perform(context)
    cam_width = LaunchConfiguration('cam_width').perform(context)
    cam_height = LaunchConfiguration('cam_height').perform(context)
    # Di mode gt gambar kamera tidak dipakai siapa pun; kamera kecil membuat
    # Webots jauh lebih ringan (diukur: 0.72x -> 0.93x waktu nyata tanpa GUI).
    default_size = ('640', '360') if vision == 'yolo' else ('64', '36')
    cam_width = int(cam_width or default_size[0])
    cam_height = int(cam_height or default_size[1])
    if vision not in ('gt', 'yolo'):
        raise RuntimeError(f"vision harus 'gt' atau 'yolo', bukan '{vision}'")

    webots_pkg = get_package_share_directory('op3_webots_ros2')
    manager_pkg = get_package_share_directory('op3_manager')

    if world_name == 'field':
        world = _render_field_world(cam_width, cam_height)
        # Dua robot <extern> di world ini, jadi tiap controller harus menyebut robotnya.
        op3_env = {'WEBOTS_CONTROLLER_URL': f'ipc://{WEBOTS_PORT}/op3'}
    elif world_name == 'empty':
        world = os.path.join(webots_pkg, 'worlds', 'robotis_op3_extern.wbt')
        op3_env = {}
    else:
        raise RuntimeError(f"world harus 'field' atau 'empty', bukan '{world_name}'")

    actions = [
        WebotsLauncher(world=world, gui=gui, port=WEBOTS_PORT),
        Node(
            package='op3_webots_ros2',
            executable='op3_extern_controller',
            output='screen',
            parameters=[{'gain_file_path': webots_pkg + '/resource/op3_pid_gain_default.yaml'}],
            additional_env=op3_env,
        ),
        Node(
            package='op3_manager',
            executable='op3_manager',
            output='screen',
            parameters=[{
                'simulation': True,
                'simulation_robot_name': 'robotis_op3',
                'offset_file_path': manager_pkg + '/config/offset.yaml',
                'robot_file_path': manager_pkg + '/config/OP3.robot',
                'init_file_path': manager_pkg + '/config/dxl_init_OP3.yaml',
                'device_name': '/dev/ttyUSB0',
                'action_file_path': _build_sim_motion_file(),
            }],
            # open_cr_module publish IMU nol ke topic ini karena gagal buka
            # serial asli. Dibelokkan ke _raw (diabaikan) supaya tidak rebutan
            # publisher dengan imu_yaw_bridge yang mengisi data IMU asli.
            # Joint state dari Webots lewat imu_yaw_bridge yang membuang pesan
            # NaN pertama; kalau tidak, goal kepala dan lengan bisa NaN selamanya.
            remappings=[
                ('/robotis/open_cr/imu', '/robotis/open_cr/imu_raw'),
                ('/robotis_op3/joint_states', '/robotis_op3/joint_states_valid'),
            ],
        ),
        Node(
            package='aroc_sim_bridge',
            executable='imu_yaw_bridge',
            output='screen',
        ),
    ]

    if world_name == 'field':
        webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
        controller_lib = os.path.join(webots_home, 'lib', 'controller')
        actions.append(Node(
            package='aroc_sim_bridge',
            executable='ball_tracker',
            output='screen',
            parameters=[{
                'cam_width': cam_width,
                'cam_height': cam_height,
                'publish_obj_detect': vision == 'gt',
            }],
            additional_env={
                'WEBOTS_HOME': webots_home,
                'WEBOTS_CONTROLLER_URL': f'ipc://{WEBOTS_PORT}/ball_tracker',
                'PYTHONPATH': os.path.join(controller_lib, 'python') + os.pathsep
                + os.environ.get('PYTHONPATH', ''),
                'LD_LIBRARY_PATH': controller_lib + os.pathsep
                + os.environ.get('LD_LIBRARY_PATH', ''),
            },
        ))

    if vision == 'yolo' and model:
        try:
            get_package_share_directory('GoNasional')
        except Exception:
            raise RuntimeError(
                "package 'GoNasional' tidak ditemukan; source dulu workspace-nya: "
                'source ~/SKRIPSI/BismilahNasional/install/local_setup.bash')
        if not os.path.isfile(os.path.join(model, 'metadata.yaml')):
            raise RuntimeError(f'folder model OpenVINO tidak valid (tanpa metadata.yaml): {model}')
        actions.append(Node(
            package='GoNasional',
            executable='vision',
            output='screen',
            parameters=[{
                'model_path': model,
                'show_window': False,
                'imgsz': _model_imgsz(model),
            }],
            remappings=[('/usb_cam_node/image_raw', '/robotis_op3/camera/image_raw')],
            # NumPy 2.x di ~/.local menutupi NumPy 1.x sistem yang dipakai saat
            # cv_bridge dan torch dikompilasi; dist-packages sistem didahulukan.
            additional_env={
                'PYTHONPATH': '/usr/lib/python3/dist-packages' + os.pathsep
                + os.environ.get('PYTHONPATH', ''),
            },
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Tampilkan window 3D Webots (true/false).'),
        DeclareLaunchArgument(
            'world', default_value='field',
            description="'field' (lapangan KRSBI + bola + ball_tracker) atau 'empty'."),
        DeclareLaunchArgument(
            'vision', default_value='gt',
            description="'gt': /obj_detect dari ground truth; 'yolo': dari GoNasional vision."),
        # 16:9 seperti C920 robot (1280x720); main_vision memampatkan ke 450x337
        # sama seperti di robot. Resolusi lebih kecil supaya render Webots ringan.
        DeclareLaunchArgument(
            'model', default_value='',
            description='Folder model OpenVINO; dengan vision:=yolo menjalankan GoNasional vision.'),
        DeclareLaunchArgument(
            'cam_width', default_value='',
            description='Kosong: 640 untuk vision:=yolo, 64 untuk vision:=gt.'),
        DeclareLaunchArgument('cam_height', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
