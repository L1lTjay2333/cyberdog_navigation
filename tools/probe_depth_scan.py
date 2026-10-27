"""Read-only comparison of native and depth-derived LaserScan."""
import json,math,time
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(args=[])
n=rclpy.create_node('web_depth_scan_probe',enable_rosout=False,start_parameter_services=False)
rows={};subs=[]
def receive(name,m):
    central=sorted(v for i,v in enumerate(m.ranges) if math.isfinite(v) and
                   abs(m.angle_min+i*m.angle_increment)<=math.radians(7.5))
    rows[name]={'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'frame':m.header.frame_id,
                'valid_bins':sum(math.isfinite(v) for v in m.ranges),'central_count':len(central),
                'central_min_m':central[0] if central else None,
                'central_median_m':central[len(central)//2] if central else None}
for name in ('scan','web_depth_scan'):
    subs.append(n.create_subscription(LaserScan,'/mi_desktop_48_b0_2d_7b_0b_4a/'+name,
                                     lambda m,key=name:receive(key,m),qos_profile_sensor_data))
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
print(json.dumps({'robot_now':time.time(),'scans':rows}),flush=True)
n.destroy_node();rclpy.shutdown()
