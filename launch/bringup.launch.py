"""Bringup simulasi OP3 lengkap: Webots + op3_manager (mode simulasi) + bridge.

Pakai:
    ros2 launch aroc_sim_bridge bringup.launch.py                 # lapangan KRSBI + bola
    ros2 launch aroc_sim_bridge bringup.launch.py world:=empty    # world kosong bawaan ROBOTIS
    ros2 launch aroc_sim_bridge bringup.launch.py gui:=false

Di world:=field, ball_tracker publish ground-truth /obj_detect, /vision/hasil_servo,
dan /vision/bola_posisi, jadi GoNasional cukup:
    ros2 run GoNasional state
(trigonocak dan vision tidak perlu dijalankan.)
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


def _render_field_world():
    """Isi path RobotisOp3.proto ke template world, tulis ke file sementara."""
    bridge_pkg = get_package_share_directory('aroc_sim_bridge')
    op3_proto = os.path.join(
        get_package_share_directory('op3_webots_ros2'), 'protos', 'RobotisOp3.proto')
    with open(os.path.join(bridge_pkg, 'worlds', 'krsbi_field.wbt.in')) as f:
        world = f.read().replace('__OP3_PROTO__', op3_proto)
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


def _launch_setup(context):
    gui = LaunchConfiguration('gui')
    world_name = LaunchConfiguration('world').perform(context)

    webots_pkg = get_package_share_directory('op3_webots_ros2')
    manager_pkg = get_package_share_directory('op3_manager')

    if world_name == 'field':
        world = _render_field_world()
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
            remappings=[
                ('/robotis/open_cr/imu', '/robotis/open_cr/imu_raw'),
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
            additional_env={
                'WEBOTS_HOME': webots_home,
                'WEBOTS_CONTROLLER_URL': f'ipc://{WEBOTS_PORT}/ball_tracker',
                'PYTHONPATH': os.path.join(controller_lib, 'python') + os.pathsep
                + os.environ.get('PYTHONPATH', ''),
                'LD_LIBRARY_PATH': controller_lib + os.pathsep
                + os.environ.get('LD_LIBRARY_PATH', ''),
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
        OpaqueFunction(function=_launch_setup),
    ])
