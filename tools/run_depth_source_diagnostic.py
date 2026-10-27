"""Read-only six-second depth diagnostic through the existing SSH profile."""
import json
import subprocess
from pathlib import Path

root=Path(__file__).resolve().parents[1]
source='import sys,types\n'
for name in ('cloud_geometry','depth_geometry','depth_corridor'):
    source+='mod=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=mod\n'
    source+='exec('+repr((root/'tools'/ (name+'.py')).read_text(encoding='utf-8'))+',mod.__dict__)\n'
source+=(root/'tools/check_depth_sources.py').read_text(encoding='utf-8')
result=subprocess.run(['ssh','-F',str(root/'config/readonly_ssh_config'),'-o','BatchMode=yes','-o','ConnectTimeout=5',
    'cyberdog-current-nx','source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],
    input=source,text=True,encoding='utf-8',capture_output=True,timeout=25)
print(result.stdout)
if result.returncode:raise RuntimeError(result.stderr[-1000:])
