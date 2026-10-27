"""Read-only live sensor inventory; never starts sensors or publishes controls."""
import json
import shlex
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/_ssh_vendor'))
import paramiko

REMOTE = r'''
import time,json,math,rclpy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data
rclpy.init(args=[])
n=rclpy.create_node('sensor_candidates_readonly',enable_rosout=False,start_parameter_services=False)
subs=[];records={}
try:
 start=time.monotonic()
 while time.monotonic()-start<2:rclpy.spin_once(n,timeout_sec=.1)
 for topic,types in n.get_topic_names_and_types():
  if not any(word in topic.lower() for word in ('scan','tof','camera','depth','points')):continue
  if not types:continue
  entry={'type':types[0],'publishers':n.count_publishers(topic),'received':0}
  records[topic]=entry
  def cb(m,e=entry):
   e['received']+=1;e['last_receive']=time.time()
   if hasattr(m,'header'):
    e['frame']=m.header.frame_id
    e['stamp']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
   if hasattr(m,'encoding'):
    e['image']={'width':m.width,'height':m.height,'encoding':m.encoding,'bytes':len(m.data)}
   elif hasattr(m,'point_step'):
    e['cloud']={'width':m.width,'height':m.height,'bytes':len(m.data)}
   elif hasattr(m,'ranges'):
    v=[float(x) for x in m.ranges if math.isfinite(x) and m.range_min<=x<=m.range_max]
    e['scan']={'valid':len(v),'min':min(v) if v else None}
   else:
    data=message_to_ordereddict(m)
    e['payload']={k:v for k,v in data.items() if k not in ('header','data')}
  try:subs.append(n.create_subscription(get_message(types[0]),topic,cb,qos_profile_sensor_data))
  except Exception as ex:entry['error']=str(ex)
 start=time.monotonic()
 while time.monotonic()-start<6:rclpy.spin_once(n,timeout_sec=.1)
 print(json.dumps({'robot_now':time.time(),'control_requests':0,'sensors':records}),flush=True)
finally:n.destroy_node();rclpy.shutdown()
'''

c = paramiko.SSHClient()
c.load_host_keys(str(Path.home() / '.ssh/known_hosts'))
c.set_missing_host_key_policy(paramiko.RejectPolicy())
try:
    c.connect('192.168.1.123', username='mi', key_filename=str(ROOT / 'config/navigation_access'),
              look_for_keys=False, allow_agent=False, timeout=5)
    inp, out, err = c.exec_command('source /etc/mi/ros2_env.conf; python3 -u -c ' +
                                 shlex.quote('import sys;exec(sys.stdin.read())'), timeout=25)
    inp.write(REMOTE)
    inp.flush()
    inp.channel.shutdown_write()
    result = out.read().decode('utf-8', 'replace')
    errors = err.read().decode('utf-8', 'replace')
    (ROOT / 'evidence/sensor_candidates_20261008.json').write_text(result, encoding='utf-8')
    print(result)
    print(errors, file=sys.stderr)
    if out.channel.recv_exit_status() != 0:
        raise RuntimeError('Sensor inventory failed')
finally:
    c.close()
