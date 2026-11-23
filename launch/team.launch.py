"""Tiga OP3 satu tim di satu lapangan Webots, untuk uji team play.

Pakai:
    ros2 launch aroc_sim_bridge team.launch.py                  # 3 robot + stack GoNasional
    ros2 launch aroc_sim_bridge team.launch.py gui:=false
    ros2 launch aroc_sim_bridge team.launch.py soccer:=false    # hanya simulasi + op3_manager

Tiap robot jalan di ROS_DOMAIN_ID sendiri (domain_base + nomor pemain, default
11, 12, 13), persis seperti tiga robot asli: op3_manager, bridge, dan stack
GoNasional masing-masing tidak saling lihat di ROS. Satu-satunya jalur antar
robot adalah UDP team_comm, sama seperti di lapangan. Untuk melihat satu robot:
    ROS_DOMAIN_ID=12 ros2 topic echo /behavior/role

Vision ground truth (ball_tracker per robot, kamera kecil) dan landmark
sim_landmarks, supaya tiga robot muat di satu laptop. Pemain 1 kiper.
"""

import importlib.util
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher

WEBOTS_PORT = '1234'
CAM_W, CAM_H = 64, 36

# (x, y, theta) awal tiap pemain; tim menyerang ke +x.
START_POSES = [
    (-4.1, 0.0, 0.0),     # 1: kiper
    (-1.6, 1.2, 0.0),
    (-2.2, -1.0, 0.0),
    (-1.0, -2.0, 0.0),
]

ROBOT_BLOCK = """DEF OP3_{k} RobotisOp3 {{
  translation {x} {y} 0.282665
  rotation 0 0 1 {theta}
  name "op3_{k}"
  controller "<extern>"
  supervisor TRUE
  cameraWidth {w}
  cameraHeight {h}
  bodySlot [
    InertialUnit {{
    }}
  ]
  headSlot [
    Pose {{
      translation 0.015348 -0.019 0.04975
    }}
  ]
}}
Robot {{
  name "ball_tracker_{k}"
  controller "<extern>"
  supervisor TRUE
  synchronization FALSE
}}
"""


def _bringup_module():
    """Helper motion file dari bringup.launch.py (satu sumber untuk page tendang sim)."""
    path = os.path.join(get_package_share_directory('aroc_sim_bridge'),
                        'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('aroc_bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _render_world(poses, ball):
    bridge_pkg = get_package_share_directory('aroc_sim_bridge')
    op3_proto = os.path.join(
        get_package_share_directory('op3_webots_ros2'), 'protos', 'RobotisOp3.proto')
    robots = ''.join(ROBOT_BLOCK.format(k=k, x=x, y=y, theta=theta, w=CAM_W, h=CAM_H)
                     for k, (x, y, theta) in enumerate(poses, start=1))
    with open(os.path.join(bridge_pkg, 'worlds', 'krsbi_team.wbt.in')) as f:
        world = (f.read().replace('__OP3_PROTO__', op3_proto)
                 .replace('__BALL__', f'{ball[0]} {ball[1]}')
                 .replace('__ROBOTS__', robots))
    out = tempfile.NamedTemporaryFile('w', suffix='_krsbi_team.wbt', delete=False)
    out.write(world)
    out.close()
    return out.name


def _launch_setup(context):
    n = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= n <= len(START_POSES):
        raise RuntimeError(f'robots harus 1..{len(START_POSES)}, bukan {n}')
    domain_base = int(LaunchConfiguration('domain_base').perform(context))
    soccer = LaunchConfiguration('soccer').perform(context) == 'true'
    goalie = LaunchConfiguration('goalie').perform(context)
    ball = [float(v) for v in LaunchConfiguration('ball').perform(context).split(',')]
    poses = START_POSES[:n]

    webots_pkg = get_package_share_directory('op3_webots_ros2')
    manager_pkg = get_package_share_directory('op3_manager')
    motion_file = _bringup_module()._build_sim_motion_file()
    webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
    controller_lib = os.path.join(webots_home, 'lib', 'controller')

    actions = [WebotsLauncher(world=_render_world(poses, ball),
                              gui=LaunchConfiguration('gui'), port=WEBOTS_PORT)]
    for k, pose in enumerate(poses, start=1):
        domain = {'ROS_DOMAIN_ID': str(domain_base + k)}
        actions += [
            Node(
                package='op3_webots_ros2', executable='op3_extern_controller',
                name='op3_extern_controller', output='screen',
                # Vision ground truth: kamera tidak dirender (jauh lebih ringan).
                parameters=[{'gain_file_path':
                             webots_pkg + '/resource/op3_pid_gain_default.yaml',
                             'camera_period_ms': 0}],
                additional_env={**domain,
                                'WEBOTS_CONTROLLER_URL': f'ipc://{WEBOTS_PORT}/op3_{k}'}),
            Node(
                package='op3_manager', executable='op3_manager', output='screen',
                parameters=[{
                    'simulation': True,
                    'simulation_robot_name': 'robotis_op3',
                    'offset_file_path': manager_pkg + '/config/offset.yaml',
                    'robot_file_path': manager_pkg + '/config/OP3.robot',
                    'init_file_path': manager_pkg + '/config/dxl_init_OP3.yaml',
                    'device_name': '/dev/ttyUSB0',
                    'action_file_path': motion_file,
                }],
                # Sama dengan bringup.launch.py (lihat komentar di sana).
                remappings=[
                    ('/robotis/open_cr/imu', '/robotis/open_cr/imu_raw'),
                    ('/robotis_op3/joint_states', '/robotis_op3/joint_states_valid'),
                ],
                additional_env=domain),
            Node(package='aroc_sim_bridge', executable='imu_yaw_bridge', output='screen',
                 additional_env=domain),
            Node(
                package='aroc_sim_bridge', executable='ball_tracker', output='screen',
                parameters=[{'cam_width': CAM_W, 'cam_height': CAM_H,
                             'robot_def': f'OP3_{k}', 'publish_obj_detect': True}],
                additional_env={
                    **domain,
                    'WEBOTS_HOME': webots_home,
                    'WEBOTS_CONTROLLER_URL': f'ipc://{WEBOTS_PORT}/ball_tracker_{k}',
                    'PYTHONPATH': os.path.join(controller_lib, 'python') + os.pathsep
                    + os.environ.get('PYTHONPATH', ''),
                    'LD_LIBRARY_PATH': controller_lib + os.pathsep
                    + os.environ.get('LD_LIBRARY_PATH', ''),
                }),
        ]
        if soccer:
            soccer_launch = os.path.join(get_package_share_directory('GoNasional'),
                                         'launch', 'aroc_soccer.launch.py')
            actions.append(GroupAction(scoped=True, actions=[
                SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_base + k)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(soccer_launch),
                    launch_arguments={
                        'sim': 'true', 'referee': 'false', 'team': 'true',
                        'player': str(k), 'goalie': goalie,
                        'start_pose': ','.join(str(v) for v in pose),
                    }.items()),
            ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Tampilkan window 3D Webots (true/false).'),
        DeclareLaunchArgument('robots', default_value='3',
                              description='Jumlah OP3 satu tim (1-4).'),
        DeclareLaunchArgument('soccer', default_value='true',
                              description='false: tanpa stack GoNasional (hanya sim + op3_manager).'),
        DeclareLaunchArgument('goalie', default_value='1',
                              description='Nomor pemain kiper (0 = tanpa kiper).'),
        DeclareLaunchArgument('ball', default_value='-0.6,0.4',
                              description="Posisi awal bola 'x,y' (m)."),
        DeclareLaunchArgument('domain_base', default_value='10',
                              description='Robot k memakai ROS_DOMAIN_ID domain_base + k.'),
        OpaqueFunction(function=_launch_setup),
    ])
