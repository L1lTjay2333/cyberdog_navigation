"""One stock mapping operation; no velocity or navigation motion goals."""
import json,time,shutil,tempfile
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from protocol.action import Navigation
from protocol.srv import AlgoTaskStatus,StopAlgoTask,SensorOperation
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
n=rclpy.create_node('computer_mapping_operation',enable_rosout=False,start_parameter_services=False)
def emit(event,**data):print(json.dumps(dict(event=event,unix=time.time(),**data)),flush=True)
def call(name,typ,req,timeout=30):
 c=n.create_client(typ,NS+'/'+name)
 try:
  if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Service unavailable: '+name)
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
  if not f.done() or f.result() is None:raise RuntimeError('Uncertain response; do not retry: '+name)
  return f.result()
 finally:n.destroy_client(c)
def ensure_lidar_scan():
 # A running mapping task does not prove that the lidar is publishing frames.
 last_scan=[0.0]
 sub=n.create_subscription(LaserScan,NS+'/scan',lambda _:last_scan.__setitem__(0,time.monotonic()),qos_profile_sensor_data)
 try:
  def wait_scan(seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:
    rclpy.spin_once(n,timeout_sec=.2)
    if last_scan[0] and time.monotonic()-last_scan[0]<1.0:return True
   return False
  if wait_scan(2.0):return
  req=SensorOperation.Request();req.sensor_id=req.ID_LIDAR;req.operation=req.OPR_START
  response=call('sensor_operation',SensorOperation,req,15)
  # Some stock firmware replies success=False/code=0 even when frames resume.
  emit('LIDAR_START_REQUEST',success=bool(response.success),code=int(response.code))
  if not wait_scan(7.0):raise RuntimeError('雷达没有恢复扫描帧；建图不能视为就绪')
  emit('LIDAR_SCAN_RECOVERED')
 finally:n.destroy_subscription(sub)
def ensure_nonempty_mapping_grid():
 grid=[None]
 def receive(message):
  if message.header.frame_id=='laser_odom' and message.info.width>0 and message.info.height>0:
   grid[0]=message
 sub=n.create_subscription(OccupancyGrid,NS+'/map',receive,10)
 try:
  until=time.monotonic()+5.0
  while time.monotonic()<until and grid[0] is None:rclpy.spin_once(n,timeout_sec=.2)
  if grid[0] is None:raise RuntimeError('当前地图仍为空，拒绝保存；先恢复扫描帧')
 finally:n.destroy_subscription(sub)
try:
 s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
 emit('STATUS',status=int(s.status),code=int(s.code))
 if REQUEST['action']=='start':
  # An explicit new-scene request may follow completed localization. Reset
  # only that confirmed state; never stop a navigation/following/mapping task.
  if s.status==8 and s.code==0 and REQUEST.get('stop_localization') is True:
   old=Path('/home/mi/mapping')
   if old.is_symlink() or not old.is_dir():raise RuntimeError('Saved-map backup target is invalid')
   dest=Path(tempfile.mkdtemp(prefix='computer_map_backup_',dir='/home/mi'))/'mapping'
   shutil.copytree(str(old),str(dest));emit('BACKUP',path=str(dest))
   req=StopAlgoTask.Request();req.task_id=req.ALGO_TASK_ALL;req.map_name=''
   r=call('stop_algo_task',StopAlgoTask,req,40)
   emit('LOCALIZATION_STOP_RESPONSE',result=int(r.result),code=int(r.code))
   if r.result!=r.SUCCESS or r.code!=0:raise RuntimeError('Localization reset rejected; do not start mapping')
   s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
   if s.status!=101 or s.code!=0:raise RuntimeError('Localization reset did not return idle')
  # Stock CheckStatusValid explicitly accepts idle or laser-localization failure.
  if s.status not in (101,6) or s.code!=0:raise RuntimeError('Start scanning requires idle or failed laser localization/code0')
  if 'depth_camera' in (REQUEST.get('mapping_sensor'),REQUEST.get('navigation_sensor')):
   if s.status==6:
    req=StopAlgoTask.Request();req.task_id=0
    r=call('stop_algo_task',StopAlgoTask,req,40)
    if r.result!=0 or r.code!=0:raise RuntimeError('Failed localization reset rejected')
   ensure_depth_slam(n,call,emit,navigation_only=REQUEST.get('mapping_sensor')!='depth_camera')
  def idle_guard():
   current=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request(),5)
   if current.status not in (101,6) or current.code!=0:raise RuntimeError('Another task started during camera handoff')
  prepare_stock_camera('camera',n,NS,idle_guard,emit)
  if REQUEST.get('mapping_sensor')=='lidar':ensure_lidar_scan()
  old=Path('/home/mi/mapping')
  if old.exists():
   dest=Path(tempfile.mkdtemp(prefix='computer_map_backup_',dir='/home/mi'))/'mapping'
   shutil.copytree(str(old),str(dest));emit('BACKUP',path=str(dest))
  a=ActionClient(n,Navigation,NS+'/start_algo_task')
  if not a.wait_for_server(timeout_sec=5):raise RuntimeError('Mapping action unavailable')
  g=Navigation.Goal();g.nav_type=g.NAVIGATION_TYPE_START_MAPPING;g.outdoor=False;g.poses=[];g.map_name=REQUEST['map_name']
  f=a.send_goal_async(g,feedback_callback=lambda x:emit('FEEDBACK',code=x.feedback.feedback_code,message=x.feedback.feedback_msg))
  rclpy.spin_until_future_complete(n,f,timeout_sec=10)
  if not f.done() or f.result() is None:raise RuntimeError('Uncertain mapping acknowledgement; do not retry')
  if not f.result().accepted:raise RuntimeError('Mapping request rejected')
  emit('ACK',accepted=True)
  until=time.monotonic()+25
  while time.monotonic()<until:
   rclpy.spin_once(n,timeout_sec=.2)
   s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request(),8)
   if s.status==5:
    if REQUEST.get('mapping_sensor')=='lidar':ensure_lidar_scan()
    emit('SCANNING',map_name=REQUEST['map_name']);break
  else:raise RuntimeError('Mapping readiness not confirmed; task may remain active')
 else:
  if s.status!=5:raise RuntimeError('Save requires active mapping status 5')
  if REQUEST.get('mapping_sensor')=='lidar':ensure_nonempty_mapping_grid()
  req=StopAlgoTask.Request();req.task_id=req.ALGO_TASK_MAPPING;req.map_name=REQUEST['map_name']
  r=call('stop_algo_task',StopAlgoTask,req,40)
  emit('SAVE_RESPONSE',result=int(r.result),code=int(r.code),map_name=req.map_name)
  if r.result!=r.SUCCESS or r.code!=0:raise RuntimeError('Stock map save rejected')
  s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
  if s.status!=101 or s.code!=0:raise RuntimeError('Map save did not return to idle; no map files changed')
  emit('MAP_FILES_SELECTED',**archive_other_maps('/home/mi/mapping',REQUEST['map_name']))
  emit('SAVED',status=int(s.status),map_name=req.map_name)
except Exception as e:
 emit('ERROR',message=str(e));raise
finally:n.destroy_node();rclpy.shutdown()
