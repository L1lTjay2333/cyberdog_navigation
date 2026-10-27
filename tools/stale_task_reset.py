"""Operator-requested recovery of a completed but stale stock navigation task."""
import json
import subprocess
import time
from pathlib import Path


def reset_stale_navigation(root, snapshot, nav, mapping, manual_started, exploring, confirmed):
    if confirmed is not True:
        raise ValueError('请先确认机器狗已实际停稳，再手动重置')
    if nav.get('active') or mapping.get('active') or manual_started or exploring.get('active'):
        raise ValueError('还有定位、导航、建图或遥控会话，先结束会话')
    now = time.time()
    task = snapshot.get('task') or {}
    if (snapshot.get('connection') != 'connected' or
            not 0 <= now - snapshot.get('local_received_unix', 0) < 3 or
            task.get('status') != 1 or
            not 0 <= snapshot.get('emitted_unix', 0) - task.get('received_unix', 0) < 2):
        raise ValueError('仅允许在实时原厂状态为导航执行中、且本页无活动任务时重置')
    root = Path(root)
    code = (root / 'tools/reset_completed_navigation.py').read_text(encoding='utf-8')
    command = ['ssh', '-F', str(root / 'config/readonly_ssh_config'),
               'cyberdog-current-nx',
               'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"']
    try:
        result = subprocess.run(command, input=code, text=True, encoding='utf-8',
                                capture_output=True, timeout=65)
    except subprocess.TimeoutExpired as exc:
        raise ValueError('重置响应超时，结果不确定；请检查原厂任务状态，不要重复点击') from exc
    rows = []
    for line in result.stdout.splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    if result.returncode or not any(row.get('event') == 'COMPLETED_NAVIGATION_RESET' for row in rows):
        raise ValueError('原厂未确认清理；可能仍有活动动作、里程计不静止或服务未响应。请核对状态，勿重复点击')
    return {'ok': True, 'message': '原厂已确认清理卡住的导航任务；等待状态回到待机后，再点击「重新定位」。不会自动行走。'}
