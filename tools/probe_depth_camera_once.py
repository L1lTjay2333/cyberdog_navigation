"""Bounded existing-camera lifecycle/pointcloud trial; restore initial idle state."""
import json
import argparse
import shlex
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/_ssh_vendor'))
import paramiko

REMOTE = r'''
import json,time,rclpy
from lifecycle_msgs.srv import GetState,ChangeState
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init(args=[])
n=rclpy.create_node('depth_camera_trial',enable_rosout=False,start_parameter_services=False)
path='/mi_desktop_48_b0_2d_7b_0b_4a/camera/camera'
modified=False;old=None;initial=None;passed=False;records=[];count=[0];depth_count=[0];calibration=[None];last=[0.]
buf=Buffer();listener=TransformListener(buf,n)
def emit(kind,**data):print(json.dumps(dict(kind=kind,**data)),flush=True)
def call(name,typ,req):
 c=n.create_client(typ,path+'/'+name)
 try:
  if not c.wait_for_service(timeout_sec=3):raise RuntimeError('service unavailable: '+name)
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=12)
  if not f.done():raise RuntimeError('service timeout: '+name)
  return f.result()
 finally:n.destroy_client(c)
def state():return call('get_state',GetState,GetState.Request()).current_state
def transition(i):
 req=ChangeState.Request();req.transition.id=i
 r=call('change_state',ChangeState,req);s=state()
 emit('transition',id=i,success=r.success,state=s.id,label=s.label)
 if not r.success:raise RuntimeError('transition failed')
def setcloud(v):
 req=SetParameters.Request()
 req.parameters=[Parameter(name='pointcloud.enable',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=v))]
 r=call('set_parameters',SetParameters,req)
 if len(r.results)!=1 or not r.results[0].successful:raise RuntimeError('pointcloud parameter rejected')
 emit('pointcloud_parameter',enabled=v)
def receive(m):
 count[0]+=1
 if count[0]%5!=1:return
 d={'frame':m.header.frame_id,'width':m.width,'height':m.height,
    'age_s':(n.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)*1e-9}
 try:
  pts=decode_xyz(m,20000)
  d['finite_sampled_points']=len(pts)
  t=buf.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
  pts=transform_xyz(pts,[t.translation.x,t.translation.y,t.translation.z],
                   [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
  d['body_roi']=summarize(pts,[.1,5.,-.3,.3,.05,1.],10)
 except Exception as e:d['error']=str(e)
 records.append(d);emit('cloud',**d)
def receive_depth(m):
 depth_count[0]+=1
 if time.monotonic()-last[0]<.5:return
 last[0]=time.monotonic()
 d={'frame':m.header.frame_id,'encoding':m.encoding,'width':m.width,'height':m.height,
    'age_s':(n.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)*1e-9}
 try:
  info=calibration[0]
  if info is None:raise ValueError('Waiting for CameraInfo')
  dt=abs(Time.from_msg(info.header.stamp).nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)*1e-9
  if dt>.2:raise ValueError('Calibration message too old for depth frame')
  pts,total=depth_xyz(m,info)
  d['sampled_pixels']=total;d['finite_points']=len(pts)
  central=sorted(p[2] for p in pts if abs(p[0]/p[2])<.1 and abs(p[1]/p[2])<.1)
  d['central_optical_z_median_m']=central[len(central)//2] if central else None
  d['valid_depth_fraction']=len(pts)/float(total) if total else 0
  t=buf.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
  d['optical_to_body']={'translation':[t.translation.x,t.translation.y,t.translation.z],
                        'rotation':[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]}
  pts=transform_xyz(pts,d['optical_to_body']['translation'],d['optical_to_body']['rotation'])
  d['body_roi']=summarize(pts,[.1,5.,-.3,.3,.05,1.],10)
 except Exception as e:d['error']=str(e)
 records.append(d);emit('depth_derived_cloud',**d)
try:
 s=state();emit('initial_state',id=s.id,label=s.label)
 initial=s.id
 if s.id not in (1,2,3):raise RuntimeError('refuse trial: camera is transitioning')
 if initial!=3:
  modified=True
  if initial==1:transition(1)
  if state().id!=2:raise RuntimeError('camera not inactive after configure')
  transition(3)
 begun=time.monotonic()
 while time.monotonic()-begun<2:rclpy.spin_once(n,timeout_sec=.1)
 sub=n.create_subscription(PointCloud2,'/mi_desktop_48_b0_2d_7b_0b_4a/camera/depth/color/points',receive,qos_profile_sensor_data)
 info_sub=n.create_subscription(CameraInfo,'/mi_desktop_48_b0_2d_7b_0b_4a/camera/depth/camera_info',lambda m:calibration.__setitem__(0,m),qos_profile_sensor_data)
 depth_sub=n.create_subscription(Image,'/mi_desktop_48_b0_2d_7b_0b_4a/camera/depth/image_rect_raw',receive_depth,qos_profile_sensor_data)
 begun=time.monotonic()
 while time.monotonic()-begun<8:rclpy.spin_once(n,timeout_sec=.1)
 emit('summary',received_native_cloud=count[0],received_depth=depth_count[0],motion_requests=0,
      usable_roi_frames=sum(bool(d.get('body_roi',{}).get('valid')) and -.05<=d['age_s']<=1 for d in records))
 passed=any(bool(d.get('body_roi',{}).get('valid')) and -.05<=d['age_s']<=1 for d in records)
finally:
 if modified and KEEP_ACTIVE and passed:
  emit('left_active',state=state().id,motion_requests=0)
 elif modified:
  try:
   s=state()
   if old is not None and s.id==3:setcloud(old)
   if s.id==3:transition(4)
   if old is not None and state().id==2:setcloud(old)
   if initial==1 and state().id==2:transition(2)
   s=state();emit('restored_state',id=s.id,label=s.label,restored=s.id==initial)
  except Exception as e:emit('restore_error',error=str(e))
 n.destroy_node();rclpy.shutdown()
'''

scope = json.loads((ROOT / 'config/scope.json').read_text(encoding='utf-8'))
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--keep-active',action='store_true',help='Keep existing sensor active only after valid depth/TF trial')
args=parser.parse_args()
if scope.get('motion_enabled') or scope.get('manual_motion_enabled'):
    raise RuntimeError('Keep all motion locks disabled during sensor trial')
geometry = (ROOT / 'tools/cloud_geometry.py').read_text(encoding='utf-8')
probe = (ROOT / 'tools/obstacle_distance_probe.py').read_text(encoding='utf-8')
prefix = 'import types,sys\nh=types.ModuleType("cloud_geometry");sys.modules["cloud_geometry"]=h\n'
prefix += 'exec(' + repr(geometry) + ',h.__dict__)\n'
prefix += 'p={"__name__":"distance_probe_library"};exec(' + repr(probe) + ',p)\n'
prefix += 'decode_xyz=h.decode_xyz;transform_xyz=h.transform_xyz;summarize=p["summarize"]\n'
prefix += 'g={};exec('+repr((ROOT/'tools/depth_geometry.py').read_text(encoding='utf-8'))+',g);depth_xyz=g["depth_xyz"]\n'
prefix += 'KEEP_ACTIVE='+repr(args.keep_active)+'\n'
c = paramiko.SSHClient()
c.load_host_keys(str(Path.home() / '.ssh/known_hosts'))
c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
    c.connect('192.168.1.123', username='mi', key_filename=str(ROOT / 'config/navigation_access'),
              look_for_keys=False, allow_agent=False, timeout=5)
    inp,out,err=c.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c '+
                              shlex.quote('import sys;exec(sys.stdin.read())'),timeout=100)
    inp.write(prefix+REMOTE);inp.flush();inp.channel.shutdown_write()
    result=out.read().decode('utf-8','replace');errors=err.read().decode('utf-8','replace')
    (ROOT/'evidence/depth_camera_trial_20261008.jsonl').write_text(result,encoding='utf-8')
    (ROOT/'evidence/depth_camera_trial_stderr_20261008.txt').write_text(errors,encoding='utf-8')
    print(result);print(errors,file=sys.stderr)
    if out.channel.recv_exit_status()!=0:raise RuntimeError('Camera trial failed; inspect restoration evidence')
finally:c.close()
