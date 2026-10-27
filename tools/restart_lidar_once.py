"""One isolated stock lidar STOP/START, no motion/map/parameter commands."""
import json
import shlex
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko
source=r'''
import json,rclpy,time
from protocol.srv import SensorOperation,AlgoTaskStatus
rclpy.init(args=[])
n=rclpy.create_node('handoff_lidar_restart_once',enable_rosout=False,start_parameter_services=False)
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
def call(name,typ,req,timeout):
 c=n.create_client(typ,NS+'/'+name)
 if not c.wait_for_service(timeout_sec=5):raise RuntimeError('service unavailable: '+name)
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done() or f.result() is None:raise RuntimeError('response uncertain: '+name)
 return f.result()
def operation(op):
 req=SensorOperation.Request();req.sensor_id=req.ID_LIDAR;req.operation=op
 r=call('sensor_operation',SensorOperation,req,15)
 print(json.dumps({'operation':op,'sensor_id':req.ID_LIDAR,'success':r.success,'code':r.code}),flush=True)
 if not r.success or r.code!=0:raise RuntimeError('lidar operation failed')
try:
 s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request(),8)
 print(json.dumps({'task':s.status,'code':s.code}),flush=True)
 if s.code!=0 or s.status not in (101,6,8):raise RuntimeError('task not quiescent; no restart sent')
 try:operation(SensorOperation.Request.OPR_STOP)
 finally:operation(SensorOperation.Request.OPR_START)
finally:n.destroy_node();rclpy.shutdown()
'''
c=paramiko.SSHClient();c.load_host_keys(str(Path.home()/'.ssh/known_hosts'));c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 c.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 inp,out,err=c.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())'),timeout=40)
 inp.write(source);inp.flush();inp.channel.shutdown_write()
 output=out.read().decode('utf-8','replace');errors=err.read().decode('utf-8','replace')
 (ROOT/'evidence/lidar_restart_once_20261008.txt').write_text(output+errors,encoding='utf-8')
 print(output);print(errors,file=sys.stderr)
 if out.channel.recv_exit_status()!=0:raise RuntimeError('Restart failed; inspect recorded replies before any retry')
finally:c.close()
