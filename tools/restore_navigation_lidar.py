"""Restore only stock Nav2 obstacle inputs to the saved lidar configuration.

This does not change maps, localization, manual control, or the camera pipeline.
The remote process restart is permitted only while the stock algorithm is idle.
Run without --apply for a read-only preflight.
"""
import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROBOT_CODE = r'''
import json, os, signal, subprocess, time
from pathlib import Path
import rclpy, yaml
from protocol.srv import AlgoTaskStatus

NS = '/mi_desktop_48_b0_2d_7b_0b_4a'
PACKAGE = Path('/home/mi/web_depth_mapping_20261011')
BINARIES = {
    '/opt/ros2/cyberdog/lib/nav2_controller/controller_server': 'controller',
    '/opt/ros2/cyberdog/lib/nav2_planner/planner_server': 'planner',
}

def emit(**row):
    print(json.dumps(row), flush=True)

def scan_topic(path, kind):
    data = yaml.safe_load(path.read_text())
    costmap = 'local_costmap' if kind == 'controller' else 'global_costmap'
    return data[NS.lstrip('/')][costmap][costmap]['ros__parameters']['obstacle_layer']['scan']['topic']

rclpy.init(args=[])
node = rclpy.create_node('web_navigation_lidar_restore', enable_rosout=False,
                         start_parameter_services=False)
client = node.create_client(AlgoTaskStatus, NS + '/algo_task_status')

def require_idle():
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('Stock task service unavailable')
    future = client.call_async(AlgoTaskStatus.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=8)
    if not future.done() or future.result() is None:
        raise RuntimeError('Stock task status uncertain')
    result = future.result()
    if result.code != 0 or result.status != 101:
        raise RuntimeError('Stock task is not idle 101; refusing process restart')

def restart(kind, pid, old_args, new_args):
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 8
    while Path('/proc/' + str(pid)).exists() and time.monotonic() < deadline:
        time.sleep(.1)
    if Path('/proc/' + str(pid)).exists():
        raise RuntimeError(kind + ' did not stop cleanly')
    with (PACKAGE / (kind + '_lidar_restore.log')).open('ab') as log:
        proc = subprocess.Popen(new_args, stdin=subprocess.DEVNULL,
                                stdout=log, stderr=log, start_new_session=True)
    time.sleep(1)
    if proc.poll() is not None:
        with (PACKAGE / (kind + '_restore_rollback.log')).open('ab') as log:
            subprocess.Popen(old_args, stdin=subprocess.DEVNULL,
                             stdout=log, stderr=log, start_new_session=True)
        raise RuntimeError(kind + ' failed to start; prior process relaunched')
    return proc.pid

try:
    require_idle()
    plans = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        except (OSError, UnicodeError):
            continue
        if not args or args[0] not in BINARIES:
            continue
        kind = BINARIES[args[0]]
        if kind in plans or '__ns:=' + NS not in args or '--params-file' not in args:
            raise RuntimeError('Duplicate or unexpected ' + kind + ' process')
        index = args.index('--params-file') + 1
        current = Path(args[index])
        original = PACKAGE / (kind + '_original.yaml')
        if not original.is_file() or original.is_symlink():
            raise RuntimeError('Original configuration missing: ' + kind)
        if scan_topic(original, kind) not in ('scan', '/scan'):
            raise RuntimeError('Saved configuration is not native lidar: ' + kind)
        topic = scan_topic(current, kind)
        if topic not in ('scan', '/scan', NS + '/web_depth_scan'):
            raise RuntimeError('Unexpected active scan topic: ' + topic)
        replacement = list(args)
        replacement[index] = str(original)
        plans[kind] = (int(entry.name), args, replacement, topic)
    if set(plans) != {'controller', 'planner'}:
        raise RuntimeError('Both stock Nav2 processes are required')
    emit(event='PREFLIGHT', topics={kind: plan[3] for kind, plan in plans.items()},
         apply=APPLY)
    if APPLY:
        for kind in ('controller', 'planner'):
            require_idle()
            pid, old_args, replacement, topic = plans[kind]
            if topic in ('scan', '/scan'):
                emit(event='ALREADY_LIDAR', node=kind, pid=pid)
                continue
            new_pid = restart(kind, pid, old_args, replacement)
            emit(event='LIDAR_RESTORED', node=kind, pid=new_pid)
        require_idle()
        emit(event='RESTORE_COMPLETE', map_unchanged=True, motion_commands=0)
finally:
    node.destroy_client(client)
    node.destroy_node()
    rclpy.shutdown()
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Restart idle stock Nav2 nodes with saved lidar input')
    args = parser.parse_args()
    command = [
        'ssh', '-F', str(ROOT / 'config/readonly_ssh_config'), '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=5', 'cyberdog-current-nx',
        'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"',
    ]
    code = 'APPLY=' + repr(args.apply) + '\n' + ROBOT_CODE
    result = subprocess.run(command, input=code, text=True, encoding='utf-8',
                            capture_output=True, timeout=45)
    print(result.stdout, end='')
    if result.returncode:
        raise RuntimeError(result.stderr[-1200:])

if __name__ == '__main__':
    main()
