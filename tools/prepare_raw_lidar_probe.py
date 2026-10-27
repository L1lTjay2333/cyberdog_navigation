"""Stage/compile an isolated diagnostic only. Does not open the lidar."""
import json,shlex,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko
c=paramiko.SSHClient();c.load_host_keys(str(Path.home()/'.ssh/known_hosts'));c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 c.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 _,out,err=c.exec_command('mktemp -d /home/mi/lidar_sdk_probe_20261008.XXXXXX',timeout=5)
 path=out.read().decode().strip()
 if not path.startswith('/home/mi/lidar_sdk_probe_20261008.') or '/' in path[len('/home/mi/'):]:raise RuntimeError('Unexpected staging path')
 with c.open_sftp() as s:s.put(str(ROOT/'tools/raw_lidar_probe.cpp'),path+'/raw_lidar_probe.cpp')
 cmd='g++ -std=c++11 -O1 -I/usr/local/include '+shlex.quote(path+'/raw_lidar_probe.cpp')+' /usr/local/lib/libydlidar_sdk.a -pthread -ldl -lrt -o '+shlex.quote(path+'/raw_lidar_probe')
 _,out,err=c.exec_command(cmd,timeout=40)
 output=out.read().decode('utf-8','replace')+err.read().decode('utf-8','replace');status=out.channel.recv_exit_status()
 (ROOT/'evidence/raw_lidar_compile_20261008.txt').write_text(output,encoding='utf-8')
 print(output)
 if status:raise RuntimeError('Compile failed; stock sensor untouched')
 (ROOT/'evidence/raw_lidar_stage_20261008.json').write_text(json.dumps({'directory':path,'binary':path+'/raw_lidar_probe','compiled':True}),encoding='utf-8')
 print('Compiled only: '+path+'/raw_lidar_probe')
finally:c.close()
