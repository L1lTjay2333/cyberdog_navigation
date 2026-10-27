"""Passive scan/TF capture for physical angle and self-return investigation."""
import json
import shlex
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/_ssh_vendor'))
import paramiko

source=(ROOT/'tools/cloud_geometry.py').read_text(encoding='utf-8')+'\n'+r'''
import json,math,time,rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
rclpy.init(args=[])
node=rclpy.create_node('handoff_scan_geometry_readonly',enable_rosout=False,start_parameter_services=False)
scans=[];buffer=Buffer();listener=TransformListener(buffer,node)
sub=node.create_subscription(LaserScan,'/mi_desktop_48_b0_2d_7b_0b_4a/scan',lambda m:scans.append(m),qos_profile_sensor_data)
begun=time.monotonic()
try:
 while time.monotonic()-begun<3:rclpy.spin_once(node,timeout_sec=.1)
 if not scans:raise RuntimeError('No scan received')
 s=scans[-1];rows=[];sectors=[]
 t=buffer.lookup_transform('base_link',s.header.frame_id,rclpy.time.Time())
 p,q=t.transform.translation,t.transform.rotation
 for i,r in enumerate(s.ranges):
  if not math.isfinite(r) or not s.range_min<=r<=s.range_max:continue
  a=s.angle_min+i*s.angle_increment;a=math.atan2(math.sin(a),math.cos(a))
  xyz=transform_xyz([(r*math.cos(a),r*math.sin(a),0.)],[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])[0]
  rows.append({'angle_deg':round(math.degrees(a),1),'range_m':round(r,3),'base_xyz':[round(v,3) for v in xyz],
               'inside_stock_footprint':-.23<=xyz[0]<=.25 and abs(xyz[1])<=.13})
 for angle in range(-180,180,15):
  values=sorted(row['range_m'] for row in rows if abs(math.degrees(math.atan2(math.sin(math.radians(row['angle_deg']-angle)),math.cos(math.radians(row['angle_deg']-angle)))))<=7.5)
  sectors.append({'angle_deg':angle,'count':len(values),'min':values[0] if values else None,'median':values[len(values)//2] if values else None})
 print(json.dumps({'robot_now':time.time(),'scan_stamp':s.header.stamp.sec+s.header.stamp.nanosec/1e9,'frame':s.header.frame_id,'laser_to_base':{'translation':[p.x,p.y,p.z],'rotation':[q.x,q.y,q.z,q.w]},'valid_count':len(rows),'nearest':sorted(rows,key=lambda r:r['range_m'])[:25],'sectors':sectors}),flush=True)
finally:node.destroy_node();rclpy.shutdown()
'''
client=paramiko.SSHClient()
client.load_host_keys(str(Path.home()/'.ssh/known_hosts'))
client.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
 client.connect('192.168.1.123',username='mi',key_filename=str(ROOT/'config/navigation_access'),look_for_keys=False,allow_agent=False,timeout=5)
 inp,out,err=client.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())'),timeout=15)
 inp.write(source);inp.flush();inp.channel.shutdown_write()
 text=out.read().decode('utf-8','replace')
 if '--no-save' not in sys.argv:
  (ROOT/'evidence/scan_geometry_20261008.json').write_text(text,encoding='utf-8')
 print(text)
 print(err.read().decode('utf-8','replace'),file=sys.stderr)
finally:client.close()
