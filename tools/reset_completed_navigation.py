"""Robot side: recover a terminal goal stuck in NAVIGATION or STOPPINGTASK."""
import json,math,time,rclpy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile,DurabilityPolicy
from protocol.srv import AlgoTaskStatus,StopAlgoTask
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
n=rclpy.create_node('web_completed_navigation_reset',enable_rosout=False,start_parameter_services=False)
qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
seen=[]
sub=n.create_subscription(GoalStatusArray,NS+'/start_algo_task/_action/status',lambda m:seen.append(m),qos)
odom=[]
odom_sub=n.create_subscription(Odometry,NS+'/odom_out',
    lambda m:odom.append((time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,
        2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w))),10)
def call(name,kind,request):
 c=n.create_client(kind,NS+'/'+name)
 try:
  if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Service missing')
  f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=45)
  if not f.done() or f.result() is None:raise RuntimeError('Uncertain response; do not retry')
  return f.result()
 finally:n.destroy_client(c)
try:
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 if not seen or any(s.status in (1,2,3) for s in seen[-1].status_list):raise RuntimeError('Active action exists; no reset')
 recent=[p for p in odom if time.monotonic()-p[0]<1.5]
 if len(recent)<6 or recent[-1][0]-recent[0][0]<.8:
  raise RuntimeError('Fresh stationary odometry unavailable; no reset')
 if (math.hypot(recent[-1][1]-recent[0][1],recent[-1][2]-recent[0][2])>.03 or
     abs(math.atan2(math.sin(recent[-1][3]-recent[0][3]),math.cos(recent[-1][3]-recent[0][3])))>.08):
  raise RuntimeError('Robot is moving; no reset')
 state=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
 if state.status not in (1,103) or state.code!=0:
  raise RuntimeError('No completed-navigation stale state')
 req=StopAlgoTask.Request();req.task_id=0
 reply=call('stop_algo_task',StopAlgoTask,req)
 if reply.result!=0 or reply.code!=0:raise RuntimeError('Reset rejected')
 until=time.monotonic()+5
 while time.monotonic()<until:
  after=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
  if after.code==0 and after.status not in (1,103):break
  time.sleep(.4)
 else:raise RuntimeError('Stop accepted but manager remains in STOPPINGTASK/NAVIGATION')
 print(json.dumps({'event':'COMPLETED_NAVIGATION_RESET','before':state.status,
   'after':after.status,'result':reply.result,'code':reply.code}),flush=True)
finally:n.destroy_node();rclpy.shutdown()
