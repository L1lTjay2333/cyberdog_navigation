"""Authorized one-shot SDK/ROS comparison, with mandatory stock restart."""
import json,shlex,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko
stage=json.loads((ROOT/'evidence/raw_lidar_stage_20261008.json').read_text())
binary=stage['binary']
if not binary.startswith('/home/mi/lidar_sdk_probe_20261008.') or not binary.endswith('/raw_lidar_probe'):raise RuntimeError('Untrusted diagnostic path')
scope=json.loads((ROOT/'config/scope.json').read_text())
if scope.get('motion_enabled') or scope.get('manual_motion_enabled'):raise RuntimeError('Motion must remain locked')
source='BINARY='+repr(binary)+'\n'+r'''
import json,math,os,signal,subprocess,time,rclpy
from protocol.srv import AlgoTaskStatus,SensorOperation
from protocol.msg import MotionStatus
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(args=[])
n=rclpy.create_node('handoff_raw_lidar_comparison',enable_rosout=False,start_parameter_services=False)
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
scans=[];motion=[];sdk=None;switched=False
subs=[n.create_subscription(LaserScan,NS+'/scan',lambda m:scans.append(m),qos_profile_sensor_data),
      n.create_subscription(MotionStatus,NS+'/motion_status',lambda m:motion.append(m),10)]
def emit(**row):print(json.dumps(row,allow_nan=False),flush=True)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def call(name,typ,request,seconds=10):
 c=n.create_client(typ,NS+'/'+name)
 if not c.wait_for_service(timeout_sec=4):raise RuntimeError('Missing service '+name)
 f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
 if not f.done() or f.result() is None:raise RuntimeError('Uncertain response '+name)
 return f.result()
def operation(op):
 req=SensorOperation.Request();req.sensor_id=req.ID_LIDAR;req.operation=op
 r=call('sensor_operation',SensorOperation,req)
 emit(event='SENSOR_REPLY',operation=op,success=r.success,code=r.code,
      note='Actual scan cessation/resumption checked independently')
def record_scan(m,label):
 pts=[]
 for i,v in enumerate(m.ranges):
  if math.isfinite(v) and m.range_min<=v<=m.range_max and v>0:
   pts.append([m.angle_min+i*m.angle_increment,float(v)])
 emit(event=label,stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,points=pts)
def interrupted(signum,frame):raise RuntimeError('Interrupted; restore stock acquisition')
signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
try:
 status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
 emit(event='TASK_STATUS',status=status.status,code=status.code)
 if status.status!=101 or status.code!=0:raise RuntimeError('Task not idle; no takeover sent')
 spin(1.5)
 if not scans or not motion:raise RuntimeError('Missing scan/motion; no takeover sent')
 m=motion[-1]
 emit(event='MOTION_STATUS',motion_id=m.motion_id,switch_status=m.switch_status,ori_error=m.ori_error,footpos_error=m.footpos_error)
 if m.motion_id!=112 or m.switch_status!=0 or m.ori_error or m.footpos_error:raise RuntimeError('Robot not in verified quiet standing state')
 for scan in scans[-8:]:record_scan(scan,'ROS_BEFORE')
 switched=True
 operation(SensorOperation.Request.OPR_STOP)
 # Drain queued data then require the ROS scan to stop before accessing its port.
 spin(.4);scans.clear();spin(.5)
 if scans:raise RuntimeError('Stock scan still running; SDK not started')
 # Confirm no original process retains the serial descriptor.
 pids=subprocess.check_output(['pgrep','-f','^/opt/ros2/cyberdog/lib/sensor_manager']).decode().split()
 if len(pids)!=1:raise RuntimeError('Cannot identify stock sensor process')
 for fd in os.listdir('/proc/'+pids[0]+'/fd'):
  try:target=os.readlink('/proc/'+pids[0]+'/fd/'+fd)
  except FileNotFoundError:continue
  if target=='/dev/ttyTHS0':raise RuntimeError('Stock still holds serial port; SDK not started')
 emit(event='SERIAL_EXCLUSIVE_CHECK',stock_scan_stopped=True,stock_fd_released=True)
 sdk=subprocess.Popen([BINARY],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 stdout,stderr=sdk.communicate(timeout=25)
 emit(event='SDK_FINISH',returncode=sdk.returncode,stdout=stdout.decode('utf-8','replace'),stderr=stderr.decode('utf-8','replace'))
 if sdk.returncode:raise RuntimeError('SDK diagnostic failed')
finally:
 if sdk is not None and sdk.poll() is None:sdk.kill();sdk.wait(timeout=3)
 if switched:
  try:
   operation(SensorOperation.Request.OPR_START)
   scans.clear();spin(3)
   emit(event='RESTORE_CHECK',fresh_scan_count=len(scans),restored=bool(scans))
   for scan in scans[-8:]:record_scan(scan,'ROS_AFTER')
  finally:n.destroy_node();rclpy.shutdown()
 else:n.destroy_node();rclpy.shutdown()
'''
c=paramiko.SSHClient();c.load_host_keys(str(Path.home()/'.ssh/known_hosts'));c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 c.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 inp,out,err=c.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())'),timeout=65)
 inp.write(source);inp.flush();inp.channel.shutdown_write()
 output=out.read().decode('utf-8','replace');errors=err.read().decode('utf-8','replace')
 (ROOT/'evidence/raw_lidar_comparison_20261008.jsonl').write_text(output,encoding='utf-8')
 (ROOT/'evidence/raw_lidar_comparison_stderr_20261008.txt').write_text(errors,encoding='utf-8')
 for line in output.splitlines():
  item=json.loads(line)
  if item.get('event') not in ('ROS_BEFORE','ROS_AFTER','SDK_FINISH'):print(json.dumps(item))
  if item.get('event')=='SDK_FINISH':print(json.dumps({'event':'SDK_FINISH','returncode':item['returncode'],'raw_frames':item['stdout'].count('RAW_JSON ')}))
 print(errors,file=sys.stderr)
 if out.channel.recv_exit_status()!=0:raise RuntimeError('Diagnostic failed; inspect recovery before retrying')
finally:c.close()
