"""Run one isolated ArduCopter SITL flight and record its ROS odometry."""

import csv
import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[1]


def draw_track(samples, destination):
    if not samples:
        return
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.8))
    fig.subplots_adjust(left=0.10, right=0.98, bottom=0.23, top=0.90)
    ax.set_aspect('equal', adjustable='box')
    ax.add_patch(plt.Circle((10, 0), 2, facecolor='#db7b43', alpha=0.85, label='Cylinder'))
    ax.plot([p[0] for p in samples], [p[1] for p in samples], color='#20659c',
            linewidth=2, label='SITL odometry')
    ax.scatter([0], [0], color='#21805c', s=55, label='Start')
    ax.scatter([20], [0], marker='*', color='#393451', s=130, label='Goal')
    ax.set(xlabel='East (m)', ylabel='North (m)', title='ArduCopter SITL | horizontal flight path')
    ax.margins(x=0.06, y=0.18)
    ax.grid(alpha=0.2)
    fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=4, frameon=False)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def run():
    run_id = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder = WORKSPACE / 'runtime' / 'runs' / run_id
    folder.mkdir(parents=True)
    print(f'Run directory: {folder}', flush=True)
    firmware = WORKSPACE / 'runtime' / 'firmware'
    params = Path(get_package_share_directory('uav_nav')) / 'config' / 'avoidance.parm'
    executables = Path(get_package_prefix('uav_nav')) / 'lib' / 'uav_nav'
    sitl_cmd = [str(firmware / 'arducopter'), '--model', 'quad', '--speedup', '1',
                '--home=-35.363261,149.165230,584,0',
                '--defaults', f'{firmware / "copter.parm"},{params}']

    os.environ['ROS_LOG_DIR'] = str(folder / 'ros')
    rclpy.init()
    monitor = rclpy.create_node('sitl_recorder')
    processes = []
    samples = []
    phase = None

    def on_status(msg):
        nonlocal phase
        if msg.data != phase:
            phase = msg.data
            timestamp = datetime.datetime.now().astimezone().isoformat(timespec='milliseconds')
            with (folder / 'mission.log').open('a') as journal:
                journal.write(f'{timestamp} {phase}\n')

    def start(command, **kwargs):
        process = subprocess.Popen(command, cwd=folder, start_new_session=True, **kwargs)
        processes.append(process)
        return process

    try:
        with (folder / 'sitl.log').open('w') as logfile, (folder / 'trajectory.csv').open('w', newline='') as record:
            csv_out = csv.writer(record, lineterminator='\n')
            csv_out.writerow(['ros_time_s', 'east_m', 'north_m', 'up_m', 'phase'])

            def on_pose(msg):
                stamp = msg.header.stamp
                pos = msg.pose.pose.position
                samples.append((pos.x, pos.y))
                csv_out.writerow([f'{stamp.sec}.{stamp.nanosec:09d}', pos.x, pos.y, pos.z, phase])

            monitor.create_subscription(String, '/uav/mission_status', on_status, 10)
            monitor.create_subscription(Odometry, '/uav/odom', on_pose, qos_profile_sensor_data)
            sitl = start(sitl_cmd, stdout=logfile, stderr=subprocess.STDOUT)
            # Only connect the mission to the simulator that bound SERIAL0 for this run.
            deadline = time.monotonic() + 15
            while 'Waiting for connection' not in (folder / 'sitl.log').read_text(errors='replace'):
                if sitl.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('SITL could not open TCP 5760. Inspect sitl.log.')
                time.sleep(0.1)

            start([str(executables / 'sim_lidar')])
            start([str(executables / 'sitl_bridge'), '--ros-args', '-p', 'run_mission:=true'])
            deadline = time.monotonic() + 420
            while rclpy.ok() and phase not in ('failed', 'succeeded'):
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Mission timed out in phase {phase!r}')
                for process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'{process.args[0]} exited with status {process.returncode}')
                rclpy.spin_once(monitor, timeout_sec=0.2)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        draw_track(samples, folder / 'trajectory.png')
    print(f'Mission result: {phase or "interrupted"} | {folder}', flush=True)
    return 0 if phase == 'succeeded' else 1


if __name__ == '__main__':
    try:
        sys.exit(run())
    except (RuntimeError, OSError, KeyboardInterrupt) as exc:
        sys.exit(f'Simulation ended: {exc}')
