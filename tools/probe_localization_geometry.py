"""Robot-side read-only TF and map/laser agreement probe; no publishers."""
import json
import math
import time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener

NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node=rclpy.create_node('web_localization_geometry_probe',enable_rosout=False,start_parameter_services=False)
buffer=Buffer();listener=TransformListener(buffer,node);received={}
qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
sensor=QoSProfile(depth=1);sensor.reliability=ReliabilityPolicy.BEST_EFFORT
def map_receive(m):
    if m.header.frame_id=='map':received['map']=m
subs=[node.create_subscription(OccupancyGrid,NS+'/map',map_receive,qos),
      node.create_subscription(LaserScan,NS+'/scan',lambda m:received.update(scan=m),sensor)]
def planar(t):
    p,q=t.translation,t.rotation
    return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
try:
    end=time.monotonic()+5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    scan=received['scan'];grid=received['map']
    base=planar(buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform)
    extrinsic=planar(buffer.lookup_transform('base_link',scan.header.frame_id,rclpy.time.Time()).transform)
    origin=grid.info.origin
    ox,oy=origin.position.x,origin.position.y
    oa=2*math.atan2(origin.orientation.z,origin.orientation.w);res=grid.info.resolution
    def cell(x,y):
        dx,dy=x-ox,y-oy
        ix=int(math.floor((math.cos(oa)*dx+math.sin(oa)*dy)/res));iy=int(math.floor((-math.sin(oa)*dx+math.cos(oa)*dy)/res))
        if not(0<=ix<grid.info.width and 0<=iy<grid.info.height):return None
        return grid.data[iy*grid.info.width+ix]
    points=[]
    for i in range(0,len(scan.ranges),max(1,len(scan.ranges)//100)):
        distance=scan.ranges[i]
        if not math.isfinite(distance) or not max(.2,scan.range_min)<distance<min(8.,scan.range_max):continue
        angle=scan.angle_min+i*scan.angle_increment+extrinsic[2]
        points.append((extrinsic[0]+distance*math.cos(angle),extrinsic[1]+distance*math.sin(angle)))
    scores=[]
    for offset in (0,math.pi):
        yaw=base[2]+offset;known=matched=0
        for x,y in points:
            px=base[0]+math.cos(yaw)*x-math.sin(yaw)*y;py=base[1]+math.sin(yaw)*x+math.cos(yaw)*y
            samples=[cell(px+dx*res,py+dy*res) for dx in range(-3,4) for dy in range(-3,4)]
            valid=[v for v in samples if v is not None and v>=0]
            if valid:
                known+=1;matched+=int(any(v>=65 for v in valid))
        scores.append({'yaw_offset_deg':round(math.degrees(offset)),'known_endpoints':known,'matched_endpoints':matched,
                       'matched_fraction':matched/float(known) if known else None})
    frames={}
    for target,source in (('base_link','base_link_leg'),('base_link','camera_depth_optical_frame')):
        try:frames[target+'<-'+source]=planar(buffer.lookup_transform(target,source,rclpy.time.Time()).transform)
        except Exception as exc:frames[target+'<-'+source]=str(exc)[:90]
    sectors=[]
    for center in range(-75,90,15):
        values=sorted(r for i,r in enumerate(scan.ranges) if math.isfinite(r) and scan.range_min<=r<=scan.range_max
                      and abs(math.degrees(scan.angle_min+i*scan.angle_increment)-center)<7.5)
        sectors.append({'angle_deg':center,'n':len(values),'min':values[0] if values else None,
                        'median':values[len(values)//2] if values else None})
    hits=[]
    for i in range(1,101):
        distance=i*.05
        if cell(base[0]+distance*math.cos(base[2]),base[1]+distance*math.sin(base[2])) is not None and cell(base[0]+distance*math.cos(base[2]),base[1]+distance*math.sin(base[2]))>=65:
            hits.append(distance);break
    print(json.dumps({'map_pose':base,'scan_frame':scan.header.frame_id,'base_from_scan':extrinsic,
          'scan_angles':[scan.angle_min,scan.angle_max],'frames':frames,'endpoint_checks':scores,'sectors':sectors,'map_forward_hit_m':hits,
          'note':'Endpoint-only diagnostic; new obstacles and symmetric scenes can make results ambiguous'}),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
