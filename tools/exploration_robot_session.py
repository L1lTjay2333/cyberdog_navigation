"""Standalone robot-side mapping exploration; probe is default, never auto-restart.

Deployed with an extracted, reviewed manual bootstrap. No stock Navigation goals.
Initial request and 10 Hz deadman heartbeat arrive over the owned SSH stdin.
"""
import json,sys,threading,time,math
from pathlib import Path
REQUEST=json.loads(sys.stdin.readline())
SENSOR_MODE='supervised_four_way_trial'
AUXILIARY_TASK=5
MOTION_ENABLED=False
from manual_policy import *
exec((Path(__file__).resolve().parent/'exploration_bootstrap.py').read_text(encoding='utf-8'),globals())
from nav_msgs.msg import OccupancyGrid,Odometry
from unknown_goal import UnknownGoal,plan,number
from exploration_policy import sensor_problem,track,overlay_depth
from pose_history import PoseHistory,CompensatedPose,ScanOdometry
from geometry_msgs.msg import PoseStamped
from depth_corridor import observed_corridor
from cloud_geometry import FootprintObservation

scan_turn=REQUEST.get('scan_turn') is True
# Remove the expensive image subscription inherited from the manual bootstrap.
# The ROS callback only retains the newest frame; a worker performs projection.
for subscription in list(subscriptions):
    if subscription.topic_name in (NS+'/camera/depth/image_rect_raw',NS+'/camera/depth/color/points'):
        node.destroy_subscription(subscription);subscriptions.remove(subscription)
image_pending=[None];image_lock=threading.Lock()
def queue_image(msg):
    with image_lock:image_pending[0]=msg
def process_images():
    while running[0] and rclpy.ok():
        with image_lock:msg=image_pending[0];image_pending[0]=None
        if msg is None:time.sleep(.005);continue
        depth(msg)
        snapshot=health.get('exploration_depth')
        if snapshot:
            health['depth_points_base']=snapshot['points'];health['depth_stamp_ns']=snapshot['stamp']
if not scan_turn:
    subscriptions.append(node.create_subscription(Image,NS+'/camera/depth/image_rect_raw',queue_image,camera_qos))
    threading.Thread(target=process_images,daemon=True).start()

execute=REQUEST.get('execute') is True
footprint_observation=FootprintObservation()
live=[True];cancelled=[False];heartbeat=[time.monotonic()]
maps={};target=REQUEST.get('target');start_pose=None
pose_history=PoseHistory()
odom_history=PoseHistory('odom',600)
compensated_pose=CompensatedPose(pose_history,odom_history)
scan_odometry=None
pose_diagnostic={}
last_pose=None;distance_travelled=0.;begin=time.monotonic()
progress_pose=None;progress_at=begin
MAX_RUNTIME=120.;MAX_TRAVEL=5.
turn_total=0.;turn_last=None
scan_pause_at=None

def emit(event,**data):
    print(json.dumps(dict(event=event,unix=time.time(),**data),allow_nan=False),flush=True)

def read_deadman():
    try:
        for line in sys.stdin:
            try:
                item=json.loads(line)
                if item.get('cancel') is True:cancelled[0]=True
                if item.get('heartbeat') is True and item.get('operator_live') is True:heartbeat[0]=time.monotonic()
            except (ValueError,TypeError):pass
    finally:live[0]=False

def mapping_receive(msg):
    if msg.header.frame_id!='laser_odom':return
    maps['grid']={'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
        'frame':msg.header.frame_id,'received_unix':time.time(),
        'origin':{'x':msg.info.origin.position.x,'y':msg.info.origin.position.y,'qz':msg.info.origin.orientation.z,'qw':msg.info.origin.orientation.w},'data':list(msg.data)}

subscriptions.append(node.create_subscription(OccupancyGrid,NS+'/map',mapping_receive,camera_qos))
def native_pose(msg):
    p,q=msg.pose.position,msg.pose.orientation
    pose_diagnostic.update(frame=msg.header.frame_id,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,received=time.time())
    pose_history.add(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],msg.header.frame_id)
subscriptions.append(node.create_subscription(PoseStamped,NS+'/dog_pose',native_pose,camera_qos))
def native_odom(msg):
    if msg.child_frame_id!='base_link_leg':return
    p,q=msg.pose.pose.position,msg.pose.pose.orientation
    odom_history.add(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],msg.header.frame_id)
subscriptions.append(node.create_subscription(Odometry,NS+'/odom_out',native_odom,camera_qos))
threading.Thread(target=read_deadman,daemon=True).start()
def spin_sensors():
    while running[0] and rclpy.ok():rclpy.spin_once(node,timeout_sec=.01)
sensor_thread=threading.Thread(target=spin_sensors,daemon=True);sensor_thread.start()
planner=UnknownGoal();last_plan=-100.;report=None;last_feedback=-100.;last_motion=-100.
plan_lock=threading.Lock();job={'busy':False,'report':None,'error':None,'finished':-100.}
consumed=-100.
dynamic_key=None;dynamic_cache=None
def calculate(snapshot):
    result=None;error=None
    try:
        if planner.target is None:result=planner.set_target(snapshot,target)
        else:
            planner.last_planned=0;planner.last_key=None
            result=planner.report(snapshot)
    except Exception as exc:error=str(exc)
    with plan_lock:job.update(busy=False,report=result,error=error,finished=time.monotonic())
try:
    if target:
        number(target['x']);number(target['y'])
        if target.get('frame')!='laser_odom':raise ValueError('探索目标必须属于当前建图的laser_odom坐标系')
    if execute and not target and not scan_turn:raise ValueError('自动执行缺少目标')
    emit('STARTED',execute=execute,pid=__import__('os').getpid())
    ready_emitted=False
    while live[0] and rclpy.ok():
        time.sleep(.01);now=time.monotonic()
        if cancelled[0] or now-heartbeat[0]>.6:
            emit('CANCELLED',message='已取消或操作者心跳中断');break
        if now-begin>MAX_RUNTIME:raise RuntimeError('运行时间达到120秒上限')
        grid=maps.get('grid');pose=None;points=[];alignment_error=''
        if grid:
            try:
                # Stock map_builder publishes dog_pose directly even when the
                # corresponding dynamic TF edge is absent. Use its actual
                # frame and acquisition stamp, never an identity/latest TF.
                pose=scan_odometry.latest(time.time()) if scan_turn and scan_odometry else compensated_pose.latest(time.time())
                image_sample=health.get('exploration_depth')
                depth_stamp=image_sample['stamp'] if image_sample else health.get('depth_stamp_ns',0)
                if not scan_turn and depth_stamp and now-health.get('depth_time',-100)<=.5:
                    depth_pose=compensated_pose.at(depth_stamp*1e-9,time.time())
                    points=transform_xyz(image_sample['points'] if image_sample else health.get('depth_points_base',[]),[depth_pose['x'],depth_pose['y'],depth_pose['z']],depth_pose['quaternion'])
            except Exception as exc:pose=None;alignment_error=str(exc)
        problem=sensor_problem(health,now,require_depth=not scan_turn)
        if not scan_turn and not -.05<=(node.get_clock().now().nanoseconds-health.get('depth_stamp_ns',0))*1e-9<=.5:
            problem=problem or '等待同时间戳的深度障碍投影'
        if not grid or not pose:problem=problem or '等待实时建图地图、位姿及深度变换'
        if grid and time.time()-grid['received_unix']>2:problem='建图地图过期'
        if owner_conflict[0]:problem='检测到其他控制来源'
        if alignment_error and not pose:problem+='：'+alignment_error
        if problem:
            stop()
            # A short ROS scheduling gap pauses rotation with zero velocity.
            # Resume only this bounded in-place session when data is fresh;
            # prolonged gaps or all other faults still terminate the session.
            transient_scan_gap=scan_turn and scan_odometry and problem=='等待实时建图地图、位姿及深度变换：实时补扫里程计过期'
            if execute and ready_emitted and transient_scan_gap:
                if scan_pause_at is None:
                    scan_pause_at=now;emit('SCAN_PAUSED',message=problem)
                if now-scan_pause_at<=1.:continue
            if execute and ready_emitted:raise RuntimeError(problem)
            if now-begin>15 and not ready_emitted:raise RuntimeError('执行模块未就绪：'+problem)
            if now-last_feedback>.5:
                emit('WAITING',message=problem,health_task=health.get('task'),alignment_error=alignment_error,
                     pose_feedback=pose_diagnostic,pose_samples=len(pose_history.samples),
                     depth_stamp_ns=health.get('depth_stamp_ns'),depth_source=health.get('depth_source'),
                     depth_reason=health.get('depth_invalid_reason'))
                last_feedback=now
            continue
        if scan_pause_at is not None:
            emit('SCAN_RESUMED',message='实时里程计恢复，继续原地补扫');scan_pause_at=None
        footprint_observation.observe(pose)
        if not scan_turn:
            key=(id(grid),len(footprint_observation.poses),health.get('depth_stamp_ns'))
            if key!=dynamic_key:
                dynamic_cache=overlay_depth(footprint_observation.apply(grid),pose,points);dynamic_key=key
            dynamic=dynamic_cache
        if not ready_emitted:
            if target and math.hypot(target['x']-pose['x'],target['y']-pose['y'])>5:raise RuntimeError('首版探索目标距离上限5米')
            expected=REQUEST.get('expected_start')
            if expected and (expected.get('frame')!=pose['frame'] or math.hypot(expected['x']-pose['x'],expected['y']-pose['y'])>.25):raise RuntimeError('启动位姿与网页设置时不一致，请重新设置目标')
            ready_emitted=True;start_pose=dict(pose)
            if scan_turn:
                scan_odometry=ScanOdometry(start_pose,odom_history)
                start_pose['scan_started_monotonic']=now
            emit('READY',pose=pose,execute=execute,battery=health.get('battery'))
        if not execute:
            if now-last_feedback>.5:emit('PROBE',pose=pose,depth_min=health.get('depth_min'),map_cells=grid['width']*grid['height'],depth_obstacles=len(points));last_feedback=now
            if now-begin>8:break
            continue
        if scan_turn:
            if math.hypot(pose['x']-start_pose['x'],pose['y']-start_pose['y'])>.15:raise RuntimeError('原地补扫出现平移，结束')
            if turn_last is not None:
                delta=math.atan2(math.sin(pose['yaw']-turn_last),math.cos(pose['yaw']-turn_last))
                if abs(delta)>.15:raise RuntimeError('补扫朝向突变')
                turn_total+=delta
            turn_last=pose['yaw']
            if turn_total>=2*math.pi-.08:emit('SCAN_COMPLETED',radians=turn_total,pose=pose);break
            if now-start_pose['scan_started_monotonic']>42:raise RuntimeError('原地补扫超时')
            if now-last_motion>=.05:
                last_motion=now
                if not active:active=True;send(MotionServoCmd.SERVO_START)
                send(MotionServoCmd.SERVO_DATA,0.,0.,.22)
            if now-last_feedback>.2:emit('SCAN_CONTROL',radians=turn_total,pose=pose,velocity=[0.,0.,.22]);last_feedback=now
            continue
        if last_pose:distance_travelled+=math.hypot(pose['x']-last_pose['x'],pose['y']-last_pose['y'])
        if last_pose and (math.hypot(pose['x']-last_pose['x'],pose['y']-last_pose['y'])>.12 or abs(math.atan2(math.sin(pose['yaw']-last_pose['yaw']),math.cos(pose['yaw']-last_pose['yaw'])))>.15):raise RuntimeError('连续建图位姿突变')
        last_pose=dict(pose)
        if progress_pose is None or math.hypot(pose['x']-progress_pose['x'],pose['y']-progress_pose['y'])>.05 or abs(math.atan2(math.sin(pose['yaw']-progress_pose['yaw']),math.cos(pose['yaw']-progress_pose['yaw'])))>.1:
            progress_pose=dict(pose);progress_at=now
        if now-progress_at>15:raise RuntimeError('15秒无位姿进展，不继续尝试运动')
        if distance_travelled>MAX_TRAVEL:raise RuntimeError('首版探索累计路程达到5米上限')
        with plan_lock:completed=dict(job)
        if completed['finished']>consumed:
            consumed=completed['finished']
            if completed['error']:raise RuntimeError(completed['error'])
            report=completed['report']
            emit('PLAN',status=report['status'],preview=report.get('preview'),waypoint=report.get('waypoint'))
            if report['status']=='AT_TARGET':emit('ARRIVED',pose=pose,distance_travelled_m=distance_travelled);break
            if report['status'] not in ('GOAL_ROUTE','FRONTIER_ROUTE'):raise RuntimeError(report['message'])
        if now-last_plan>=.5 and not completed['busy']:
            snapshot={'connection':'connected','local_received_unix':time.time(),'map':dynamic,'pose':pose,
                'scan':{'received_unix':time.time()-(now-health.get('scan_time',-100))},'mapping':{'map_name':REQUEST.get('map_name')}}
            with plan_lock:job['busy']=True
            last_plan=now
            threading.Thread(target=calculate,args=(snapshot,),daemon=True).start()
        if report is None:stop();continue
        if now-consumed>1.:raise RuntimeError('规划结果过期')
        velocity,reason=track(dynamic,pose,report.get('preview'),health,now)
        if reason:stop()
        elif now-last_motion>=.05:
            last_motion=now
            if any(abs(v)>1e-6 for v in velocity):
                if not active:active=True;send(MotionServoCmd.SERVO_START)
                send(MotionServoCmd.SERVO_DATA,*velocity)
            else:stop()
        if now-last_feedback>.2:
            emit('CONTROL',pose=pose,velocity=list(velocity),message=reason,distance_travelled_m=distance_travelled)
            last_feedback=now
except Exception as exc:
    emit('ERROR',message=str(exc));raise
finally:
    stop()
    if execute and sent:
        send(MotionServoCmd.SERVO_DATA)
        send(MotionServoCmd.SERVO_END)
    running[0]=False;sensor_thread.join(timeout=1.)
    until=time.monotonic()+.3
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    emit('STOP_SENT' if execute and sent else 'EXITED_NO_MOTION',message='已发送零速度和结束指令，物理停车仍以现场确认为准' if execute and sent else '本会话未发布运动指令，执行器已退出')
    node.destroy_node();rclpy.shutdown()
