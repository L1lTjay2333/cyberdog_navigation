"""Read-only diagnostic transport; no motion or task-start commands."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools' / '_ssh_vendor'))
import paramiko

client = paramiko.SSHClient()
client.load_host_keys(str(Path.home() / '.ssh' / 'known_hosts'))
client.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
    client.connect('192.168.1.123', username='mi',
                   key_filename=str(ROOT / 'config' / 'navigation_access'),
                   look_for_keys=False, allow_agent=False,
                   timeout=5, auth_timeout=8, banner_timeout=8)
    command = ('hostname; source /etc/mi/ros2_env.conf; '
               'timeout 8 ros2 service call '
               '/mi_desktop_48_b0_2d_7b_0b_4a/algo_task_status '
               'protocol/srv/AlgoTaskStatus "{}"; '
               'ls -lt /home/mi/.ros/log | head -6')
    _, output, error = client.exec_command(command, timeout=15)
    print(output.read().decode('utf-8', 'replace'))
    print(error.read().decode('utf-8', 'replace'), file=sys.stderr)
finally:
    client.close()
