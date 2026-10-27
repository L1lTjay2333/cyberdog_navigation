"""Read-only stock costmap diagnostics; no parameter or lifecycle changes."""
import json
import shlex
import sys
import urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko

snapshot=json.load(urllib.request.urlopen('http://127.0.0.1:8765/state',timeout=4))
target=snapshot['selection']['goal']
source='TARGET='+repr(target)+'\n'+r'''
import json,math,time,rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
rclpy.init(args=[])
node=rclpy.create_node('handoff_costmap_readonly_probe',enable_rosout=False,start_parameter_services=False)
qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
grids=[];buffer=Buffer();listener=TransformListener(buffer,node)
subscription=node.create_subscription(OccupancyGrid,'/mi_desktop_48_b0_2d_7b_0b_4a/global_costmap/costmap',lambda m:grids.append(m),qos)
scans=[]
scan_subscription=node.create_subscription(LaserScan,'/mi_desktop_48_b0_2d_7b_0b_4a/scan',lambda m:scans.append(m),qos_profile_sensor_data)
started=time.monotonic()
try:
 while time.monotonic()-started<5:rclpy.spin_once(node,timeout_sec=.1)
 if not grids:raise RuntimeError('No stock costmap received')
 g=grids[-1];t=buffer.lookup_transform(g.header.frame_id,'base_link',rclpy.time.Time())
 p,q=t.transform.translation,t.transform.rotation
 yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 a=2*math.atan2(g.info.origin.orientation.z,g.info.origin.orientation.w)
 def cost(x,y):
  dx=x-g.info.origin.position.x;dy=y-g.info.origin.position.y
  ix=math.floor((math.cos(a)*dx+math.sin(a)*dy)/g.info.resolution)
  iy=math.floor((-math.sin(a)*dx+math.cos(a)*dy)/g.info.resolution)
  return g.data[iy*g.info.width+ix] if 0<=ix<g.info.width and 0<=iy<g.info.height else 'outside'
 sectors={}
 if scans:
  s=scans[-1]
  for name,center in (('front',0.),('left',math.pi/2),('right',-math.pi/2)):
   values=sorted(r for i,r in enumerate(s.ranges) if math.isfinite(r) and s.range_min<=r<=s.range_max and abs(math.atan2(math.sin(s.angle_min+i*s.angle_increment-center),math.cos(s.angle_min+i*s.angle_increment-center)))<math.pi/6)
   sectors[name]={'count':len(values),'min':values[0] if values else None,'median':values[len(values)//2] if values else None,'under_0_5m':sum(r<.5 for r in values)}
 print(json.dumps({'frame':g.header.frame_id,'stamp':g.header.stamp.sec+g.header.stamp.nanosec/1e9,'robot_now':time.time(),'pose':{'x':p.x,'y':p.y,'yaw':yaw},'start_cost':cost(p.x,p.y),'goal_cost':cost(TARGET['x'],TARGET['y']),'ahead':[(d,cost(p.x+d*math.cos(yaw),p.y+d*math.sin(yaw))) for d in (.2,.4,.6,.8,1.,1.2)],'occupied_count':sum(v>=99 for v in g.data),'unknown_count':sum(v<0 for v in g.data),'scan_sectors':sectors}),flush=True)
finally:node.destroy_node();rclpy.shutdown()
'''
client=paramiko.SSHClient()
client.load_host_keys(str(Path.home()/'.ssh/known_hosts'))
client.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 client.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 command='source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')
 inp,out,err=client.exec_command(command,timeout=15)
 inp.write(source);inp.flush();inp.channel.shutdown_write()
 text=out.read().decode('utf-8','replace')
 print(text)
 (ROOT/'evidence/costmap_probe_20261008.txt').write_text(text,encoding='utf-8')
 print(err.read().decode('utf-8','replace'),file=sys.stderr)
finally:client.close()
