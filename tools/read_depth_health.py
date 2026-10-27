"""Read-only depth diagnostic: subscribe for six seconds, never send motion."""
import json,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source='import sys,types\n'
for name in ('cloud_geometry','depth_geometry','depth_corridor','front_cloud'):
    code=(ROOT/('tools/'+name+'.py')).read_text(encoding='utf-8')
    source+='m=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=m\nexec('+repr(code)+',m.__dict__)\n'
source+='''
import rclpy,time,json
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from cloud_geometry import decode_xyz,transform_xyz
from depth_geometry import depth_xyz,manual_sample_stride
from front_cloud import forward_points_distance
rclpy.init();n=rclpy.create_node('depth_geometry_readonly_probe',enable_rosout=False)
b=Buffer();listener=TransformListener(b,n);calibration=[None];rows=[];counts={'image':0,'cloud':0,'tf_fail':0}
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
started=time.monotonic();last=[0.]
ns='/mi_desktop_48_b0_2d_7b_0b_4a/camera/depth/'
def inspect(m,kind):
 counts[kind]+=1
 if len(rows)>=12 or time.monotonic()-started<1 or time.monotonic()-last[0]<.2:return
 last[0]=time.monotonic()
 try:
  start=time.monotonic();stamp=Time.from_msg(m.header.stamp);arrival_age=float(n.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
  if kind=='image':
   if calibration[0] is None:return
   points,total=depth_xyz(m,calibration[0],stride=manual_sample_stride(m.width,m.height))
  else:points=[p for p in decode_xyz(m,20000) if p[2]>.01];total=min(m.width*m.height,20000)
  t=b.lookup_transform('base_link',m.header.frame_id,stamp).transform
  points=transform_xyz(points,[t.translation.x,t.translation.y,t.translation.z],[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
  corridor=[p[0] for p in points if p[0]>=.1 and abs(p[1])<=.35 and .03<=p[2]<=1]
  row={'kind':kind,'arrival_age_s':round(arrival_age,3),'processing_ms':round((time.monotonic()-start)*1000,1),'age_s':round(float(n.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9,3),'valid_fraction':round(len(points)/float(total),3),'all_forward_roi_count':len(corridor),'nearest_m':float(min(corridor)) if corridor else None}
  try:row['policy_distance_m']=forward_points_distance(points,total,len(points)/float(total),[t.translation.x,t.translation.y,t.translation.z])[0]
  except ValueError as exc:row['policy_error']=str(exc)
  rows.append(row)
 except Exception as exc:counts['tf_fail']+=1
subs=[n.create_subscription(CameraInfo,ns+'camera_info',lambda m:calibration.__setitem__(0,m),qos),n.create_subscription(Image,ns+'image_rect_raw',lambda m:inspect(m,'image'),qos),n.create_subscription(PointCloud2,ns+'color/points',lambda m:inspect(m,'cloud'),qos)]
end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
print(json.dumps({'counts':counts,'samples':rows}),flush=True)
n.destroy_node();rclpy.shutdown()
'''
if __name__=='__main__':
    command=['ssh','-F',str(ROOT/'config/readonly_ssh_config'),'-o','BatchMode=yes','-o','ConnectTimeout=5','cyberdog-current-nx','source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
    result=subprocess.run(command,input=source,encoding='utf-8',capture_output=True,timeout=18)
    print(result.stdout.strip() or 'No depth diagnostic response')
    if result.returncode:print('Diagnostic failed, SSH/ROS process exit:',result.returncode)
