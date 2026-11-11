"""Python 3.6 compatible, passive stock ROS map/status monitor over stdout."""
import json
import base64
import zlib
import math
import time
import threading
from collections import deque
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan,Image,CameraInfo
from tf2_ros import Buffer, TransformListener
from protocol.msg import AlgoTaskStatus
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped,TransformStamped
from scan_distance import report_scan
from depth_geometry import depth_xyz
from depth_preview import depth_preview
from cloud_geometry import FootprintObservation

NS = '/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node = rclpy.create_node('flat_navigation_readonly_map', enable_rosout=False, start_parameter_services=False)
tf_buffer = Buffer()
mapping_pose_buffer = Buffer()
mapping_pose_history = deque(maxlen=300)
tf_listener = TransformListener(tf_buffer,node)
state = {'map': None, 'pose': None, 'task': None, 'odom': None, 'cloud': None, 'tf': {}, 'started_unix': time.time()}
footprint_observation=FootprintObservation()
def battery_receive(m):
    state['battery']={'percent':int(m.batt_soc),'wired_charging':bool(m.power_wired_charging),
        'wireless_charging':bool(m.power_wp_charging),'received_unix':time.time()}
maps_by_frame = {}
try:
    from saved_grid import load_saved_grid
    maps_by_frame['map']=load_saved_grid('/home/mi/mapping',SAVED_MAP_NAME)
except Exception:
    pass # Missing preview never substitutes a different map.
last_cloud_decode=0
pending_cloud=None
depth_info=None
last_native_cloud=0
latest_cloud=None
latest_depth=None
latest_scan=None
worker_done=threading.Event()
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
sensor_qos = QoSProfile(depth=1)
sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

def map_receive(msg):
    maps_by_frame[msg.header.frame_id] = {'width': msg.info.width, 'height': msg.info.height,
                    'resolution': msg.info.resolution, 'frame': msg.header.frame_id,
                    'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y,
                               'qz': msg.info.origin.orientation.z, 'qw': msg.info.origin.orientation.w},
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
                    'received_unix': time.time(), 'data': list(msg.data)}
    if state['map'] is None:state['map']=maps_by_frame[msg.header.frame_id]

def task_receive(msg):
    if msg.task_status==5 and (state.get('task') or {}).get('status')!=5:
        mapping_pose_buffer.clear()
        mapping_pose_history.clear()
        footprint_observation.poses=[]
        maps_by_frame.pop('map',None)
    state['task'] = {'status': msg.task_status, 'sub_status': msg.task_sub_status, 'received_unix': time.time()}

def odom_receive(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    state['odom'] = {'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                     'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id, 'received_unix': time.time()}

def cloud_receive(msg):
    global latest_cloud,last_native_cloud
    last_native_cloud=time.monotonic()
    latest_cloud=msg

def process_cloud(msg):
    global last_cloud_decode,pending_cloud,last_native_cloud
    last_native_cloud=time.monotonic()
    now=time.time()
    # Map/pose still stream at 4 Hz. A display-only cloud needs less CPU and
    # transport bandwidth than obstacle sensing, which has its own process.
    if time.monotonic()-last_cloud_decode<1.:return
    last_cloud_decode=time.monotonic()
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    cloud={'points':msg.width*msg.height,'frame':msg.header.frame_id,'source_frame':msg.header.frame_id,
           'received_unix':now,'stamp':stamp,'aligned':False,'topic':NS+'/camera/depth/color/points'}
    try:
        points=[p for p in decode_xyz(msg,800) if sum(v*v for v in p)>1e-10]
        cloud['points_xyz']=points
        try:
            t=tf_buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
            body=transform_xyz(points,[t.translation.x,t.translation.y,t.translation.z],
                               [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            xs=sorted(p[0] for p in body if p[0]>=.1 and abs(p[1])<=.35 and .03<=p[2]<=1.)
            if len(xs)>=5:cloud['front_check']={'distance_m':xs[0],'points':len(xs),'stamp':stamp}
        except Exception:pass
        status=(state.get('task') or {}).get('status')
        if status in (5,8,1) and stamp>0 and abs(now-stamp)<2:
            # TF often arrives a few milliseconds after the camera frame.
            # Retry the exact acquisition timestamp after subsequent ROS spins.
            if pending_cloud is None:pending_cloud=(msg.header,cloud,points,None)
        cloud['sampled_finite_points']=len(points)
        cloud['units']='ROS_SOURCE_METERS_SCALE_REQUIRES_FIELD_CHECK'
    except Exception as exc:cloud['error']=str(exc)
    state['cloud']=cloud

def depth_info_receive(msg):
    global depth_info
    depth_info=msg

def depth_receive(msg):
    global latest_depth
    latest_depth=msg

def process_depth(msg):
    global last_cloud_decode,pending_cloud
    if time.monotonic()-last_cloud_decode<.5:return
    last_cloud_decode=time.monotonic()
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    cloud={'points':0,'frame':msg.header.frame_id,'source_frame':msg.header.frame_id,
           'received_unix':time.time(),'stamp':stamp,'aligned':False,
           'topic':NS+'/camera/depth/image_rect_raw','source_kind':'depth_image_derived',
           'units':'ROS_SOURCE_METERS_SCALE_REQUIRES_FIELD_CHECK'}
    try:
        if abs(time.time()-stamp)>1 or stamp<=0:raise ValueError('Depth data stale or clock mismatch')
        if depth_info is None:raise ValueError('Waiting for depth CameraInfo')
        # CameraInfo is calibration, not a pose sample. Stock may publish it
        # less frequently; depth_xyz validates dimensions, optical frame and P.
        # Image freshness and timestamped world TF remain mandatory separately.
        info=depth_info
        points,total=depth_xyz(msg,info,stride=8)
        points=[p for p in points if .15<=p[2]<=4.]
        cloud.update(points_xyz=points,points=len(points),sampled_finite_points=len(points),
                     sampled_pixels=total,valid_depth_fraction=len(points)/float(total) if total else 0)
        if (state.get('task') or {}).get('status') in (5,8,1) and pending_cloud is None:
            pending_cloud=(msg.header,cloud,points,(msg,info))
    except Exception as exc:cloud['error']=str(exc)
    state['cloud']=cloud

def display_transform(target,source,stamp):
    if target=='laser_odom' and (state.get('task') or {}).get('status')==5:
        # Mapping PoseStamped is already expressed in laser_odom. Interpolate
        # its full 6-DoF pose at sensor acquisition time, independent of vodom TF.
        # Use the actual timestamped laser_odom PoseStamped history. The stock
        # vodom->base_link TF may stop publishing during mapping; no alias needed.
        base=mapping_pose_buffer.lookup_transform('laser_odom','base_link',rclpy.time.Time.from_msg(stamp)).transform
        local=tf_buffer.lookup_transform('base_link',source,rclpy.time.Time.from_msg(stamp)).transform
        a,b=base.rotation,local.rotation
        out=TransformStamped();out.header.frame_id=target;out.header.stamp=stamp;out.child_frame_id=source
        p=transform_xyz([[local.translation.x,local.translation.y,local.translation.z]],
                        [base.translation.x,base.translation.y,base.translation.z],[a.x,a.y,a.z,a.w])[0]
        out.transform.translation.x,out.transform.translation.y,out.transform.translation.z=p
        q=out.transform.rotation
        q.x=a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y
        q.y=a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x
        q.z=a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
        q.w=a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z
        return out
    return tf_buffer.lookup_transform(target,source,rclpy.time.Time.from_msg(stamp))

def align_pending_cloud():
    global pending_cloud
    if pending_cloud is None:return
    header,cloud,points,depth_pair=pending_cloud
    age=time.time()-cloud['received_unix']
    if age<.15:return
    if abs(time.time()-cloud['stamp'])>2 or (state.get('task') or {}).get('status') not in (5,8,1):
        pending_cloud=None;return
    try:
        target='laser_odom' if (state.get('task') or {}).get('status')==5 else 'map'
        t=display_transform(target,header.frame_id,header.stamp)
        p,q=t.transform.translation,t.transform.rotation
        cloud['points_xyz']=transform_xyz(points,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        cloud['frame']=target;cloud['aligned']=True
        state['cloud']=cloud
        if depth_pair is not None:
            image,info=depth_pair
            state['archive_depth']=dict(event='depth',stamp=cloud['stamp'],frame=target,
                source_frame=header.frame_id,width=image.width,height=image.height,step=image.step,
                encoding=image.encoding,bigendian=bool(image.is_bigendian),
                intrinsics=[float(info.p[i]) for i in (0,5,2,6)],
                translation=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                depth_zlib_b64=base64.b64encode(zlib.compress(bytes(image.data),1)).decode('ascii'))
        if depth_pair is not None and target=='laser_odom':
            base=mapping_pose_buffer.lookup_transform(
                target,'base_link',rclpy.time.Time.from_msg(header.stamp)).transform
            extrinsic=tf_buffer.lookup_transform(
                'base_link',header.frame_id,rclpy.time.Time.from_msg(header.stamp)).transform
            def pose_record(transform):
                p,q=transform.translation,transform.rotation
                return dict(translation=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])
            state['archive_depth']['base_pose']=pose_record(base)
            state['archive_depth']['camera_extrinsic']=pose_record(extrinsic)
            state['archive_depth']['local_pose_history']=[
                pose for pose in list(mapping_pose_history)
                if cloud['stamp']-2 <= pose['stamp'] <= cloud['stamp']+.5]
            state['archive_depth']['pose_source']=NS+'/dog_pose'
            state['archive_depth']['pose_kind']='stock_local_slam'
        cloud.pop('alignment_error',None)
        pending_cloud=None
    except Exception as exc:cloud['alignment_error']=str(exc)[:160]

def pose_receive(msg):
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
    if not all(math.isfinite(v) for v in values) or norm < 1e-12:
        state['raw_pose'] = None
        return
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    state['raw_pose'] = {'x': p.x, 'y': p.y,
                     'yaw': math.atan2(2*(q.w*q.z+q.x*q.y)/norm, 1-2*(q.y*q.y+q.z*q.z)/norm),
                     'frame': msg.header.frame_id, 'received_unix': stamp,
                     'source': NS+'/dog_pose'}
    if (state.get('task') or {}).get('status')==5:
        if msg.header.frame_id=='laser_odom' and stamp>0:
            t=TransformStamped();t.header=msg.header;t.child_frame_id='base_link'
            t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
            t.transform.rotation.x=q.x/math.sqrt(norm);t.transform.rotation.y=q.y/math.sqrt(norm)
            t.transform.rotation.z=q.z/math.sqrt(norm);t.transform.rotation.w=q.w/math.sqrt(norm)
            mapping_pose_buffer.set_transform(t,'stock_dog_pose')
            mapping_pose_history.append(dict(stamp=stamp,translation=[p.x,p.y,p.z],
                quaternion=[t.transform.rotation.x,t.transform.rotation.y,
                            t.transform.rotation.z,t.transform.rotation.w]))
        footprint_observation.observe(state['raw_pose'])

def select_display_pose(status,grid,raw,previous,now):
    """Never relabel a mapping pose as navigation pose; preserve real age."""
    wanted='laser_odom' if status==5 else 'map'
    if not grid or grid['frame']!=wanted:return None
    if status==5 and raw and raw.get('frame')==wanted and 0<=now-raw['received_unix']<2:
        return dict(raw)
    if status not in (5,8,1):return None
    try:
        t=tf_buffer.lookup_transform(wanted,'base_link',rclpy.time.Time())
        stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
        p,q=t.transform.translation,t.transform.rotation
        norm=sum(v*v for v in (q.x,q.y,q.z,q.w))
        if abs(now-stamp)<2 and norm>1e-12 and all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
            return {'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm),
                    'frame':wanted,'received_unix':stamp,'source':'stock_tf_base_link'}
    except Exception:pass
    # Show the last matched position as expired; do not refresh its timestamp.
    return previous if previous and previous.get('frame')==wanted else None

def tf_receive(msg):
    for t in msg.transforms:
        state['tf'][t.child_frame_id] = {'parent': t.header.frame_id, 'received_unix': time.time()}

def path_receive(msg):
    state['path']={'frame':msg.header.frame_id,'points':[{'x':p.pose.position.x,'y':p.pose.position.y} for p in msg.poses], 'received_unix':time.time(),'kind':'stock_nav2'}

def scan_receive(msg):
    global latest_scan
    latest_scan=msg
    state['scan']={'frame':msg.header.frame_id,'received_unix':time.time(),'finite_ranges':sum(math.isfinite(x) and msg.range_min<=x<=msg.range_max for x in msg.ranges)}
    try:
        # Exact message timestamp, never latest TF as a substitute.
        transform=tf_buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
        state['scan']['distance']=report_scan(msg,transform,node.get_clock().now().nanoseconds)
    except Exception as exc:
        state['scan']['distance']={'valid':False,'reason':str(exc)[:200],'target_frame':'base_link'}

def scan_overlay(msg,status):
    target='laser_odom' if status==5 else 'map'
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    result={'frame':target,'received_unix':stamp,'points':[]}
    if status not in (5,8,1) or abs(time.time()-stamp)>1:return result
    try:
        t=display_transform(target,msg.header.frame_id,msg.header.stamp).transform
        source=[];central=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or not msg.range_min<=r<=msg.range_max:continue
            a=msg.angle_min+i*msg.angle_increment
            if abs(a)<=math.radians(7.5):central.append(r)
            if i%3==0:source.append([r*math.cos(a),r*math.sin(a),0.])
        p,q=t.translation,t.rotation
        result['points']=[{'x':v[0],'y':v[1]} for v in transform_xyz(source,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])]
        if len(central)>=5:result['central_range_m']=sorted(central)[len(central)//2]
    except Exception as exc:result['error']=str(exc)[:120]
    return result

subscriptions = [node.create_subscription(OccupancyGrid, NS+'/map', map_receive, qos),
                 node.create_subscription(Path, NS+'/plan', path_receive, sensor_qos),
                 node.create_subscription(LaserScan, NS+globals().get('SCAN_TOPIC','/scan'), scan_receive, sensor_qos),
                 node.create_subscription(PoseStamped, NS+'/dog_pose', pose_receive, sensor_qos),
                 node.create_subscription(AlgoTaskStatus, NS+'/algo_task_status', task_receive, 1),
                 node.create_subscription(Odometry, NS+'/odom_out', odom_receive, sensor_qos),
                 node.create_subscription(PointCloud2, NS+'/camera/depth/color/points', cloud_receive, sensor_qos),
                 node.create_subscription(CameraInfo, NS+'/camera/depth/camera_info', depth_info_receive, sensor_qos),
                 node.create_subscription(Image, NS+'/camera/depth/image_rect_raw', depth_receive, sensor_qos),
                 node.create_subscription(TFMessage, '/tf', tf_receive, sensor_qos),
                 node.create_subscription(TFMessage, '/tf_static', tf_receive, qos)]
try:
    from protocol.msg import BmsStatus
    subscriptions.append(node.create_subscription(BmsStatus,NS+'/bms_status',battery_receive,sensor_qos))
except ImportError:pass
def select_display_grid(maps,status):
    if status==5:return maps.get('laser_odom')
    return maps.get('map')

def display_worker():
    global latest_cloud,latest_depth
    cached_key=None;cached_grid=None;sent_grid=None
    while not worker_done.is_set():
        begun=time.monotonic()
        try:
            # Coalesce frames. Projection/JSON must not hold up ROS TF/status callbacks.
            cloud=latest_cloud;latest_cloud=None
            depth=latest_depth;latest_depth=None
            if depth is not None:
                try:state['depth_preview']=depth_preview(depth,time.time())
                except ValueError as error:state['depth_preview']={'error':str(error),'received_unix':time.time()}
            if depth is not None and (state.get('task') or {}).get('status') in (5,8,1):process_depth(depth)
            elif cloud is not None:process_cloud(cloud)
            elif depth is not None:process_depth(depth)
            align_pending_cloud()
            report=dict(state);report['tf']=dict(state['tf'])
            state.pop('archive_depth',None)
            status=(report.get('task') or {}).get('status')
            if latest_scan is not None:report['laser_overlay']=scan_overlay(latest_scan,status)
            grid=select_display_grid(maps_by_frame,status)
            poses=footprint_observation.poses
            key=(id(grid),id(poses))
            if key!=cached_key:
                cached_grid=footprint_observation.apply(grid) if grid else None
                cached_key=key
            report['map']=cached_grid
            report['pose']=select_display_pose(status,cached_grid,report.get('raw_pose'),state.get('pose'),time.time())
            state['pose']=report['pose']
            if cached_grid is sent_grid:report.pop('map',None)
            else:sent_grid=cached_grid
            report['emitted_unix']=time.time()
            print(json.dumps(report,allow_nan=False,separators=(',',':')),flush=True)
        except (BrokenPipeError,OSError):worker_done.set();return
        except Exception as exc:
            print(json.dumps({'stream_error':str(exc),'emitted_unix':time.time()}),flush=True)
        worker_done.wait(max(.01,.25-(time.monotonic()-begun)))

worker=threading.Thread(target=display_worker);worker.daemon=True;worker.start()
try:
    while rclpy.ok() and not worker_done.is_set():
        rclpy.spin_once(node,timeout_sec=.02)
finally:
    worker_done.set();worker.join(timeout=1)
    node.destroy_node()
    rclpy.shutdown()
