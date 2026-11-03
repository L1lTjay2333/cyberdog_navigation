"""Robot-side read-only image topic inventory. Python 3.6 compatible.

No services, lifecycle changes, publishers, or image export. This checks live
metadata and timestamp proximity; it cannot certify visual alignment.
"""
import json
import math
import time
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage,CameraInfo


def main():
    rclpy.init(args=[])
    node=rclpy.create_node('readonly_rgbd_source_inventory',enable_rosout=False,start_parameter_services=False)
    counts={};rows={};stamps={};subscriptions=[];found={}
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        kinds={'sensor_msgs/msg/Image':Image,'sensor_msgs/msg/CompressedImage':CompressedImage,
               'sensor_msgs/msg/CameraInfo':CameraInfo}
        topics=[(topic,kind) for topic,types in node.get_topic_names_and_types()
                for kind in types if kind in kinds and any(word in topic.lower() for word in ('camera','image','rgb','color','depth'))]
        topics=sorted(topics)
        if len(topics)>40:raise ValueError('Too many image topics; refine inventory before subscribing')
        def receive(topic,kind,msg):
            counts[topic]=counts.get(topic,0)+1
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            stamps[topic].append(stamp)
            row={'type':kind,'frame_id':msg.header.frame_id,'stamp':stamp}
            if kind.endswith('/CameraInfo'):
                row.update(width=msg.width,height=msg.height,k=list(msg.k),p=list(msg.p),
                           distortion_model=msg.distortion_model,d=list(msg.d))
            elif kind.endswith('/Image'):
                row.update(width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,bytes=len(msg.data))
            else:row.update(format=msg.format,bytes=len(msg.data))
            rows[topic]=row
        for topic,kind in topics:
            found[topic]={'type':kind,'publishers':node.count_publishers(topic)}
            stamps[topic]=deque(maxlen=32)
            subscriptions.append(node.create_subscription(kinds[kind],topic,
                lambda msg,t=topic,k=kind:receive(t,k,msg),qos_profile_sensor_data))
        end=time.monotonic()+7
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        images=[t for t,k in topics if not k.endswith('/CameraInfo') and stamps[t]]
        proximity=[]
        for a in images:
            if 'depth' not in a.lower():continue
            for b in images:
                if a==b or 'depth' in b.lower():continue
                differences=[min(abs(x-y) for y in stamps[b]) for x in stamps[a] if math.isfinite(x)]
                proximity.append({'depth':a,'other':b,'nearest_stamp_difference_s':min(differences) if differences else None})
        print(json.dumps({'topics':found,'counts':counts,'last_messages':rows,'timestamp_proximity':proximity,
                          'control_requests':0,'camera_changes':0,'rgbd_ready':False,
                          'note':'Metadata inventory only. Timestamp proximity does not prove synchronization or spatial registration.'},allow_nan=False),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
