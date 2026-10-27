"""Run teammate's read-only diagnostic over SSH, without robot file installs."""
import argparse
import json
import shlex
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--mode',choices=['scan','cloud'],required=True)
args=p.parse_args()
topic='/mi_desktop_48_b0_2d_7b_0b_4a/'+('scan' if args.mode=='scan' else 'camera/depth/color/points')
source='import sys,types\nh=types.ModuleType("cloud_geometry")\nsys.modules["cloud_geometry"]=h\n'
source+='exec('+repr((ROOT/'tools/cloud_geometry.py').read_text(encoding='utf-8'))+',h.__dict__)\n'
source+='sys.argv='+repr(['obstacle_distance_probe','--mode',args.mode,'--topic',topic,
                          '--target-frame','base_link','--seconds','5','--max-age','1',
                          '--roi','.1','5','-.3','.3','-.1','1'])+'\n'
source+=(ROOT/'tools/obstacle_distance_probe.py').read_text(encoding='utf-8')
c=paramiko.SSHClient();c.load_host_keys(str(Path.home()/'.ssh/known_hosts'));c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 c.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 inp,out,err=c.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())'),timeout=20)
 inp.write(source);inp.flush();inp.channel.shutdown_write()
 output=out.read().decode('utf-8','replace');errors=err.read().decode('utf-8','replace')
 (ROOT/('evidence/reference_distance_'+args.mode+'_20261008.jsonl')).write_text(output,encoding='utf-8')
 (ROOT/('evidence/reference_distance_'+args.mode+'_stderr_20261008.txt')).write_text(errors,encoding='utf-8')
 print(output);print(errors,file=sys.stderr)
 if out.channel.recv_exit_status()!=0:raise RuntimeError('Remote diagnostic failed')
finally:c.close()
