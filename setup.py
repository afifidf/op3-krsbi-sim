import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'aroc_sim_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.wbt*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afifi',
    maintainer_email='196213202+afifidf@users.noreply.github.com',
    description='Simulasi Webots OP3 sepak bola: lapangan, bola ground-truth, bridge OpenCR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_yaw_bridge = aroc_sim_bridge.imu_yaw_bridge:main',
            'ball_tracker = aroc_sim_bridge.ball_tracker:main',
            'place_ball = aroc_sim_bridge.place_ball:main',
            'vision_eval = aroc_sim_bridge.vision_eval:main',
            'loc_bench = aroc_sim_bridge.loc_bench:main',
            'dataset_gen = aroc_sim_bridge.dataset_gen:main',
            'field_viewer = aroc_sim_bridge.field_viewer:main',
            'team_monitor = aroc_sim_bridge.team_monitor:main',
        ],
    },
)
