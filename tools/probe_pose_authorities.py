"""Passive robot-side probe: inventory publishers and sample TF/pose values."""
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

rclpy.init(args=[])
node=rclpy.create_node('web_pose_authority_probe',enable_rosout=False,start_parameter_services=False)
rows={}

def record(key,p,q,stamp,info=None):
    # Galactic's Python callbacks do not expose MessageInfo. An inventory
    # therefore must not be presented as proof that each publisher sent TF.
    if info is not None:key=key+'@'+bytes(info.publisher_gid).hex()
    pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),stamp.sec+stamp.nanosec/1e9]
    row=rows.setdefault(key,{'count':0,'first':pose,'last':pose,'poses':[]})
    row['count']+=1;row['last']=pose
    if len(row['poses'])<100:row['poses'].append(pose)

def tf_callback(message,info=None):
    for t in message.transforms:
        if t.child_frame_id in ('vodom','base_link'):
            record(t.header.frame_id+'->'+t.child_frame_id,t.transform.translation,t.transform.rotation,t.header.stamp,info)

def pose_callback(message,info=None):
    record('dog_pose:'+message.header.frame_id,message.pose.position,message.pose.orientation,message.header.stamp,info)

subscriptions=[node.create_subscription(TFMessage,'/tf',tf_callback,qos_profile_sensor_data),
               node.create_subscription(PoseStamped,'/mi_desktop_48_b0_2d_7b_0b_4a/dog_pose',pose_callback,qos_profile_sensor_data)]
try:
    started=time.monotonic()
    while time.monotonic()-started<5:rclpy.spin_once(node,timeout_sec=.1)
    publishers={}
    for topic in ('/tf','/mi_desktop_48_b0_2d_7b_0b_4a/dog_pose'):
        for endpoint in node.get_publishers_info_by_topic(topic):
            publishers[bytes(endpoint.endpoint_gid).hex()]=endpoint.node_namespace+'/'+endpoint.node_name
    print(json.dumps({'publishers':publishers,'samples':rows}),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
