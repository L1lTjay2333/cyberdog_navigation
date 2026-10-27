"""Check actual navigation subscriptions without sending any robot action."""
import ast
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
source=(root/'tools/stock_navigation_session.py').read_text(encoding='utf-8')
tree=ast.parse(source)
names={'navigation_map_receive','wait_for_navigation_inputs'}
functions='\n'.join(ast.get_source_segment(source,n) for n in tree.body
                    if isinstance(n,ast.FunctionDef) and n.name in names)
prelude='''import time,json,rclpy
from tf2_ros import Buffer,TransformListener
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from protocol.msg import BmsStatus
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node=rclpy.create_node('web_navigation_inputs_probe',enable_rosout=False,start_parameter_services=False)
buffer=Buffer();listener=TransformListener(buffer,node)
observations={'map':None,'scan':0,'battery':None,'battery_received':0}
frames=[]
def operator_ready():pass
def emit(**row):print(json.dumps(row),flush=True)
'''
operation='''
qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
sensor=QoSProfile(depth=1);sensor.reliability=ReliabilityPolicy.BEST_EFFORT
def receive_map(message):
    frames.append(message.header.frame_id);navigation_map_receive(message)
subscriptions=[node.create_subscription(OccupancyGrid,NS+'/map',receive_map,qos),
 node.create_subscription(LaserScan,NS+'/scan',lambda m:observations.update(scan=time.monotonic()),sensor),
 node.create_subscription(BmsStatus,NS+'/bms_status',lambda m:observations.update(battery=m,battery_received=time.monotonic()),sensor)]
try:
    wait_for_navigation_inputs(False,0)
    emit(event='PASSIVE_INPUT_CHECK',map_frames=frames,selected_frame=observations['map'].header.frame_id)
finally:
    node.destroy_node();rclpy.shutdown()
'''
result=subprocess.run(['ssh','-F',str(root/'config/readonly_ssh_config'),'-o','BatchMode=yes',
                       '-o','ConnectTimeout=5','cyberdog-current-nx',
                       'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],
                      input=prelude+functions+operation,text=True,encoding='utf-8',capture_output=True,timeout=20)
rows=[]
for line in result.stdout.splitlines():
    try:rows.append(json.loads(line))
    except ValueError:continue
(root/'evidence/navigation_inputs_check_20261010.json').write_text(
    json.dumps({'returncode':result.returncode,'events':rows},indent=2),encoding='utf-8')
for row in rows:print(json.dumps(row))
if result.returncode:raise SystemExit(result.stderr[-1500:])
