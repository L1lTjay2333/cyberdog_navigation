"""Explicit authorized rollback: backup current map, restore original lidar inputs."""
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ROBOT_CODE=r'''
import json,os,signal,subprocess,time,shutil,tempfile
from pathlib import Path
import yaml,rclpy
from protocol.srv import AlgoTaskStatus,StopAlgoTask
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
PACKAGE=Path('/home/mi/web_depth_mapping_20261011')
SOURCE=Path('/home/mi/computer_map_backup_a1o788kg/mapping')
ACTIVE=Path('/home/mi/mapping')
NAME='computer_room_20261011_182516'
BINS={'/opt/ros2/cyberdog/lib/laser_slam/mapping':'mapping',
      '/opt/ros2/cyberdog/lib/laser_slam/localization':'localization',
      '/opt/ros2/cyberdog/lib/nav2_controller/controller_server':'controller',
      '/opt/ros2/cyberdog/lib/nav2_planner/planner_server':'planner'}
def emit(**data):print(json.dumps(data),flush=True)
rclpy.init(args=[])
n=rclpy.create_node('web_restore_lidar',enable_rosout=False,start_parameter_services=False)
def call(name,kind,request):
 c=n.create_client(kind,NS+'/'+name)
 try:
  if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Unavailable '+name)
  f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=45)
  if not f.done() or f.result() is None:raise RuntimeError('Uncertain response '+name)
  return f.result()
 finally:n.destroy_client(c)
def idle():
 s=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
 if s.status!=101 or s.code!=0:raise RuntimeError('Not idle; rollback not allowed')
try:
 # Validate all replacement inputs before stopping anything.
 for folder in (SOURCE,ACTIVE):
  if folder.is_symlink() or folder.resolve().parent.parent!=Path('/home') and folder==ACTIVE:
   raise RuntimeError('Unexpected map target')
  if not folder.is_dir():raise RuntimeError('Map directory missing')
 for file in (NAME+'.pgm',NAME+'.yaml',NAME+'.json','graph.pbstream','pose.txt'):
  if not (SOURCE/file).is_file() or (SOURCE/file).is_symlink():raise RuntimeError('Incomplete lidar map')
 plans=[]
 for entry in Path('/proc').iterdir():
  if not entry.name.isdigit():continue
  try:args=entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
  except (OSError,UnicodeError):continue
  if args and args[0] in BINS:
   kind=BINS[args[0]]
   if '__ns:='+NS not in args or '--params-file' not in args:raise RuntimeError('Unexpected process')
   backup=PACKAGE/(kind+'_original.yaml')
   document=yaml.safe_load(backup.read_text())
   group=document[NS.lstrip('/')]
   topic=group['map_builder' if kind=='mapping' else 'localization_node']['ros__parameters']['laser_scan_topic'] if kind in ('mapping','localization') else group['local_costmap' if kind=='controller' else 'global_costmap']['local_costmap' if kind=='controller' else 'global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic']
   if topic not in ('scan','/scan'):raise RuntimeError('Backup does not use native lidar')
   new=list(args);new[new.index('--params-file')+1]=str(backup)
   plans.append((kind,int(entry.name),args,new))
 if len(plans)!=4 or len({p[0] for p in plans})!=4:raise RuntimeError('Missing or duplicate process')
 status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
 if status.status==8 and status.code==0:
  req=StopAlgoTask.Request();req.task_id=0
  result=call('stop_algo_task',StopAlgoTask,req)
  if result.result!=0 or result.code!=0:raise RuntimeError('Localization reset failed')
 idle()
 saved=Path(tempfile.mkdtemp(prefix='computer_depth_rollback_backup_',dir='/home/mi'))
 shutil.copytree(str(ACTIVE),str(saved/'mapping'))
 staged=saved/'lidar_staged';shutil.copytree(str(SOURCE),str(staged))
 emit(event='MAP_BACKUP',path=str(saved/'mapping'))
 for kind,pid,old,new in plans:
  idle();os.kill(pid,signal.SIGTERM)
  until=time.monotonic()+8
  while Path('/proc/'+str(pid)).exists() and time.monotonic()<until:time.sleep(.1)
  if Path('/proc/'+str(pid)).exists():raise RuntimeError('Process did not stop cleanly')
  with (PACKAGE/(kind+'_lidar_restored.log')).open('ab') as log:
   proc=subprocess.Popen(new,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  time.sleep(.8)
  if proc.poll() is not None:
   with (PACKAGE/(kind+'_rollback.log')).open('ab') as log:
    subprocess.Popen(old,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
   raise RuntimeError('Restore failed; previous process restarted')
  emit(event='LIDAR_PROCESS_RESTORED',node=kind,pid=proc.pid)
 idle()
 # Exact validated directories, recoverable rename. Never delete either map.
 ACTIVE.rename(saved/'displaced_mapping')
 try:staged.rename(ACTIVE)
 except Exception:
  (saved/'displaced_mapping').rename(ACTIVE);raise
 emit(event='LIDAR_RESTORED',map_name=NAME,backup=str(saved))
finally:n.destroy_node();rclpy.shutdown()
'''
if __name__=='__main__':
 result=subprocess.run(['ssh','-F',str(ROOT/'config/readonly_ssh_config'),'-o','BatchMode=yes','cyberdog-current-nx',
  'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],input=ROBOT_CODE,text=True,encoding='utf-8',capture_output=True,timeout=95)
 print(result.stdout)
 if result.returncode:raise RuntimeError(result.stderr[-1600:])
