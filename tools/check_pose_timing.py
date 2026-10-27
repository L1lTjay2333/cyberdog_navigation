"""Read-only ROS pose, odometry and frame timing diagnostic; no publisher."""
import json,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init();node=rclpy.create_node('handoff_pose_timing_probe')
rows={};edges={};subscriptions=[]
def receive(name,m):
    p=m.pose.pose if name=='odom' else m.pose
    stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    rows[name]={'frame':m.header.frame_id,'child':getattr(m,'child_frame_id',None),
                'age':time.time()-stamp,'stamp':stamp,
                'p':[p.position.x,p.position.y,p.position.z],
                'q':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
def tf(m):
    for t in m.transforms:
        p,q=t.transform.translation,t.transform.rotation
        edges[t.child_frame_id]={'parent':t.header.frame_id,'p':[p.x,p.y,p.z],
                                'q':[q.x,q.y,q.z,q.w],'age':time.time()-t.header.stamp.sec-t.header.stamp.nanosec/1e9}
subscriptions.append(node.create_subscription(PoseStamped,NS+'/dog_pose',lambda m:receive('pose',m),qos_profile_sensor_data))
subscriptions.append(node.create_subscription(Odometry,NS+'/odom_out',lambda m:receive('odom',m),qos_profile_sensor_data))
subscriptions.append(node.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data))
subscriptions.append(node.create_subscription(TFMessage,'/tf_static',tf,10))
end=time.monotonic()+5;last=0
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.02)
    if time.monotonic()-last>1:
        print(json.dumps({'rows':rows,'edges':edges}),flush=True);last=time.monotonic()
node.destroy_node();rclpy.shutdown()
