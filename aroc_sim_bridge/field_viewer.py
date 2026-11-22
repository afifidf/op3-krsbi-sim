"""
Viewer 2D lapangan: pose robot dan bola, sebenarnya (ground truth) vs estimasi.

Jauh lebih ringan daripada RViz. Menampilkan:
  - robot sebenarnya (hijau)        /aroc_sim/robot_pose
  - estimasi lokalisasi (merah)     /localization/pose
  - partikel MCL (titik abu-abu)    /localization/particles
  - bola sebenarnya (putih)         /aroc_sim/ball_position
  - bola estimasi (oranye)          /world_model (aroc_interfaces, opsional)
  - state behavior                  /behavior/state

Pakai, di samping bringup + aroc_soccer:
    ros2 run aroc_sim_bridge field_viewer
Di robot asli (tanpa ground truth) hanya estimasi yang tampil.
`--snapshot file.png` menyimpan satu gambar setelah 5 detik lalu keluar.
"""

import math
import os
import sys
import threading


def _prefer_system_numpy():
    """matplotlib/numpy sistem; NumPy 2.x di ~/.local merusak modul yang dikompilasi untuk 1.x."""
    if os.environ.get('AROC_VIEWER_ENV') == '1':
        return
    env = dict(os.environ, AROC_VIEWER_ENV='1')
    env['PYTHONPATH'] = '/usr/lib/python3/dist-packages' + os.pathsep + env.get('PYTHONPATH', '')
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


def _yaw(q):
    return 2.0 * math.atan2(q.z, q.w)


def main():
    _prefer_system_numpy()
    snapshot = None
    if '--snapshot' in sys.argv:
        snapshot = sys.argv[sys.argv.index('--snapshot') + 1]
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import rclpy
    from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Rectangle
    from std_msgs.msg import String
    try:
        from aroc_interfaces.msg import WorldModel
    except ImportError:
        WorldModel = None

    rclpy.init()
    node = rclpy.create_node('aroc_field_viewer')
    length = node.declare_parameter('field.length', 9.0).value
    width = node.declare_parameter('field.width', 6.0).value
    goal_width = node.declare_parameter('field.goal_width', 1.6).value
    lock = threading.Lock()
    st = {'gt': None, 'est': None, 'particles': [], 'ball': None, 'ball_est': None,
          'state': '-', 'localized': None}

    def setter(key, fn):
        def cb(msg):
            with lock:
                st[key] = fn(msg)
        return cb

    node.create_subscription(PoseStamped, '/aroc_sim/robot_pose', setter(
        'gt', lambda m: (m.pose.position.x, m.pose.position.y, _yaw(m.pose.orientation))), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/localization/pose', setter(
        'est', lambda m: (m.pose.pose.position.x, m.pose.pose.position.y,
                          _yaw(m.pose.pose.orientation))), 10)
    node.create_subscription(PoseArray, '/localization/particles', setter(
        'particles', lambda m: [(p.position.x, p.position.y) for p in m.poses]), 2)
    node.create_subscription(PointStamped, '/aroc_sim/ball_position', setter(
        'ball', lambda m: (m.point.x, m.point.y)), 10)
    node.create_subscription(String, '/behavior/state', setter('state', lambda m: m.data), 10)
    if WorldModel is not None:
        def on_wm(m):
            with lock:
                st['ball_est'] = (m.ball_field.x, m.ball_field.y) if m.ball_known else None
                st['localized'] = m.localized
        node.create_subscription(WorldModel, '/world_model', on_wm, 10)

    def spin():
        try:
            rclpy.spin(node)
        except Exception:  # ExternalShutdownException saat jendela ditutup
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    fig, ax = plt.subplots(figsize=(9, 6.4))
    fig.canvas.manager.set_window_title('AROC field viewer')
    ax.set_facecolor('#2e7d32')
    hl, hw = length / 2, width / 2
    ax.add_patch(Rectangle((-hl, -hw), length, width, fill=False, ec='white', lw=2))
    ax.plot([0, 0], [-hw, hw], color='white', lw=1.5)
    ax.add_patch(Circle((0, 0), 0.75, fill=False, ec='white', lw=1.5))
    for sx, color in ((1, 'gold'), (-1, 'deepskyblue')):
        ax.plot([sx * hl, sx * hl], [-goal_width / 2, goal_width / 2], color=color, lw=6,
                solid_capstyle='butt')
    ax.text(hl - 0.1, hw + 0.15, 'gawang lawan', color='goldenrod', ha='right')
    ax.set_xlim(-hl - 0.5, hl + 0.5)
    ax.set_ylim(-hw - 0.5, hw + 0.5)
    ax.set_aspect('equal')

    particles = ax.scatter([], [], s=3, c='lightgray', alpha=0.6, label='partikel MCL')
    gt_dot, = ax.plot([], [], 'o', color='lime', ms=11, label='robot sebenarnya')
    gt_dir, = ax.plot([], [], '-', color='lime', lw=3)
    est_dot, = ax.plot([], [], 'o', color='red', ms=8, label='estimasi lokalisasi')
    est_dir, = ax.plot([], [], '-', color='red', lw=2)
    ball_dot, = ax.plot([], [], 'o', color='white', mec='black', ms=9, label='bola sebenarnya')
    ball_est, = ax.plot([], [], 'x', color='orange', ms=10, mew=3, label='bola estimasi')
    info = ax.text(-hl - 0.4, hw + 0.15, '', color='black', fontsize=10, family='monospace')
    ax.legend(loc='lower left', fontsize=8, framealpha=0.8)

    def arrow(pose, n=0.35):
        x, y, th = pose
        return [x, x + n * math.cos(th)], [y, y + n * math.sin(th)]

    def update(_):
        with lock:
            s = dict(st)
        for dot, line, pose in ((gt_dot, gt_dir, s['gt']), (est_dot, est_dir, s['est'])):
            if pose:
                dot.set_data([pose[0]], [pose[1]])
                line.set_data(*arrow(pose))
        particles.set_offsets(s['particles'] or [(math.nan, math.nan)])
        ball_dot.set_data(*([[s['ball'][0]], [s['ball'][1]]] if s['ball'] else [[], []]))
        ball_est.set_data(*([[s['ball_est'][0]], [s['ball_est'][1]]] if s['ball_est'] else [[], []]))
        text = f"behavior: {s['state']}"
        if s['localized'] is not None:
            text += f"   localized: {'ya' if s['localized'] else 'belum'}"
        if s['gt'] and s['est']:
            err = math.hypot(s['gt'][0] - s['est'][0], s['gt'][1] - s['est'][1])
            dth = math.degrees(math.atan2(math.sin(s['gt'][2] - s['est'][2]),
                                          math.cos(s['gt'][2] - s['est'][2])))
            text += f'   error lokalisasi: {err * 100:.0f} cm, {abs(dth):.1f}°'
        info.set_text(text)
        return particles, gt_dot, gt_dir, est_dot, est_dir, ball_dot, ball_est, info

    plt.tight_layout()
    if snapshot:
        import time
        time.sleep(5.0)
        update(None)
        fig.savefig(snapshot, dpi=90)
        print(snapshot)
    else:
        anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
        plt.show()
        del anim
    # Hentikan spin di thread lain dulu (shutdown), baru hancurkan node.
    rclpy.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
