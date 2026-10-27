"""Robot-side read-only 6 s camera source probe. No service or publisher."""
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from tf2_ros import Buffer,TransformListener
from depth_geometry import depth_xyz
from cloud_geometry import decode_xyz,transform_xyz
from depth_corridor import observed_corridor

NS='/mi_desktop_48_b0_2d_7b_0b_4a/camera'
rclpy.init();node=rclpy.create_node('readonly_web_depth_source_probe',enable_rosout=False,start_parameter_services=False)
buffer=Buffer();listener=TransformListener(buffer,node);info={};latest={};counts={};subscriptions=[]
def receive(key,m):
    counts[key]=counts.get(key,0)+1;latest[key]=m
for key,topic in (('raw','depth/image_rect_raw'),('aligned','aligned_depth_to_extcolor/image_raw')):
    subscriptions.append(node.create_subscription(Image,NS+'/'+topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
for key,topic in (('raw','depth/camera_info'),('aligned','aligned_depth_to_extcolor/camera_info')):
    subscriptions.append(node.create_subscription(CameraInfo,NS+'/'+topic,lambda m,k=key:info.update({k:m}),qos_profile_sensor_data))
subscriptions.append(node.create_subscription(PointCloud2,NS+'/depth/color/points',lambda m:receive('cloud',m),qos_profile_sensor_data))
end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
result={}
for key,m in latest.items():
    stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    row={'frames':counts[key],'frame':m.header.frame_id,'age_s':time.time()-stamp,'width':m.width,'height':m.height}
    try:
        if key=='cloud':
            points=decode_xyz(m,1200);total=min(m.width*m.height,1200);points=[p for p in points if p[2]>.01]
        else:
            row['encoding']=m.encoding
            if key not in info:raise ValueError('No matching CameraInfo')
            row['calibration_frame']=info[key].header.frame_id
            points,total=depth_xyz(m,info[key],stride=16)
        row['finite_fraction']=len(points)/float(total) if total else 0
        t=buffer.lookup_transform('base_link',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp)).transform
        points=transform_xyz(points,[t.translation.x,t.translation.y,t.translation.z],[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
        xs=[p[0] for p in points if p[0]>=.1 and abs(p[1])<=.35 and .03<=p[2]<=1]
        row['forward_points']=len(xs);row['forward_min_m']=min(xs) if xs else None
        row['measured_corridor_bound_m']=observed_corridor(points,[t.translation.x,t.translation.y,t.translation.z])
    except Exception as exc:row['error']=str(exc)[:180]
    result[key]=row
print(json.dumps(result),flush=True)
node.destroy_node();rclpy.shutdown()
