"""Independent depth-camera LaserScan publisher; no motion/service clients."""
import math
import time
import copy
import threading
import json
import rclpy
from sensor_msgs.msg import PointCloud2,LaserScan,Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from cloud_geometry import decode_xyz,transform_xyz
from depth_geometry import depth_xyz
from depth_scan_geometry import points_to_scan,ANGLE_MIN,ANGLE_INCREMENT,RANGE_MIN,RANGE_MAX

NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node=rclpy.create_node('web_depth_scan',enable_rosout=False,start_parameter_services=False)
buffer=Buffer();listener=TransformListener(buffer,node)
latest=[None];processed=[None]
image=[None];calibration=[None]
last_report=[0.]
publisher=node.create_publisher(LaserScan,NS+'/web_depth_scan',qos_profile_sensor_data)
camera_qos=copy.copy(qos_profile_sensor_data);camera_qos.depth=1
sub=node.create_subscription(PointCloud2,NS+'/camera/depth/color/points',lambda m:latest.__setitem__(0,m),camera_qos)
image_sub=node.create_subscription(Image,NS+'/camera/depth/image_rect_raw',lambda m:image.__setitem__(0,m),camera_qos)
info_sub=node.create_subscription(CameraInfo,NS+'/camera/depth/camera_info',lambda m:calibration.__setitem__(0,m),camera_qos)

def publish_frame(msg):
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    if stamp==processed[0] or not 0<=time.time()-stamp<.7:return False
    try:
        tf=buffer.lookup_transform('laser_frame',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
        p,q=tf.translation,tf.rotation
        if isinstance(msg,Image):
            if calibration[0] is None:raise ValueError('Camera calibration missing')
            # 640x480 @ stride 4 cost ~0.4 s on the robot and ages out
            # while the point-cloud publisher is unavailable. Preserve every
            # eighth pixel; unseen rays remain NaN, never fabricated free.
            points,_=depth_xyz(msg,calibration[0],stride=8)
        else:points=decode_xyz(msg,12000)
        xyz=transform_xyz(points,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        ranges=points_to_scan(xyz)
        if sum(math.isfinite(r) for r in ranges)<12:raise ValueError('Depth scan has fewer than 12 valid angular bins')
        scan=LaserScan();scan.header.stamp=msg.header.stamp;scan.header.frame_id='laser_frame'
        scan.angle_min=ANGLE_MIN;scan.angle_max=-ANGLE_MIN;scan.angle_increment=ANGLE_INCREMENT
        scan.scan_time=.2;scan.time_increment=0.;scan.range_min=RANGE_MIN;scan.range_max=RANGE_MAX
        # A frame that expired during conversion must never be advertised fresh.
        if time.time()-stamp>=.7:raise ValueError('Depth conversion exceeded frame age budget')
        scan.ranges=ranges;publisher.publish(scan);processed[0]=stamp
        return True
    except Exception as exc:
        if time.monotonic()-last_report[0]>2:
            node.get_logger().warning(str(exc));last_report[0]=time.monotonic()
        return False

done=threading.Event()
def worker():
    last_health=0.
    while not done.is_set():
        start=time.monotonic();published=False
        # Native cloud failure must still allow a valid depth image fallback.
        for msg in (latest[0],image[0]):
            if msg is not None and publish_frame(msg):published=True;break
        if time.monotonic()-last_health>=2:
            def age(msg):
                return None if msg is None else round(time.time()-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9,3)
            print(json.dumps({'event':'DEPTH_SCAN_HEALTH','unix':time.time(),
                'cloud_age':age(latest[0]),'image_age':age(image[0]),
                'scan_age':None if processed[0] is None else round(time.time()-processed[0],3),
                'conversion_seconds':round(time.monotonic()-start,3),'published':published}),flush=True)
            last_health=time.monotonic()
        done.wait(max(.01,.2-(time.monotonic()-start)))
thread=threading.Thread(target=worker);thread.daemon=True;thread.start()
try:rclpy.spin(node)
finally:done.set();thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
