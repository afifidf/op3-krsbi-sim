"""Controller extern robot pengganggu di world dataset_gen.

Tiap beberapa detik waktu simulasi memasang pose acak di sekitar pose siap jalan
OP3 (lengan turun, lutut ditekuk), supaya robot di dataset tidak berdiri kaku
dalam pose nol (lengan terentang). Dijalankan oleh dataset_gen, satu proses per
robot, dengan WEBOTS_CONTROLLER_URL=ipc://<port>/<nama robot>.
"""

import os
import random
import sys

# Motor RobotisOp3.proto -> (nilai pose siap jalan, rentang acak) dalam rad.
POSE = {
    'ShoulderR': (0.0, 0.5), 'ShoulderL': (0.0, 0.5),
    'ArmUpperR': (-1.2, 0.3), 'ArmUpperL': (1.2, 0.3),
    'ArmLowerR': (0.5, 0.4), 'ArmLowerL': (-0.5, 0.4),
    'PelvYR': (0.0, 0.1), 'PelvYL': (0.0, 0.1),
    'PelvR': (0.0, 0.05), 'PelvL': (0.0, 0.05),
    'LegUpperR': (0.6, 0.25), 'LegUpperL': (-0.6, 0.25),
    'LegLowerR': (-1.1, 0.4), 'LegLowerL': (1.1, 0.4),
    'AnkleR': (-0.5, 0.2), 'AnkleL': (0.5, 0.2),
    'FootR': (0.0, 0.05), 'FootL': (0.0, 0.05),
    'Neck': (0.0, 0.8), 'Head': (-0.3, 0.5),
}


def main():
    from controller import Robot
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    fixed = os.environ.get('DISTRACTOR_FIXED') == '1'   # untuk mengecek tanda sendi
    rng = random.Random(seed)
    robot = Robot()
    step = int(robot.getBasicTimeStep())
    motors = {name: robot.getDevice(name) for name in POSE}
    hold = 0
    while robot.step(step) != -1:
        hold -= step
        if hold > 0:
            continue
        hold = rng.randint(1500, 4000)
        for name, (center, spread) in POSE.items():
            motors[name].setPosition(center if fixed else center + rng.uniform(-spread, spread))


if __name__ == '__main__':
    main()
