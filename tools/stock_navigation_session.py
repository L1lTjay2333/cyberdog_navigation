"""Robot-side stock Navigation client, one goal, heartbeat and matched stop."""
# REQUEST is injected as validated JSON by the local bridge, never shell text.
import json
import math
import os
import select
import signal
import sys
import threading
import multiprocessing
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from protocol.action import Navigation
from protocol.srv import AlgoTaskStatus, StopAlgoTask, GetMapLabel, SensorOperation
from protocol.msg import BmsStatus, MotionServoCmd, MotionStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState, ChangeState
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

NS='/mi_desktop_48_b0_2d_7b_0b_4a'
# Linux robot: fork the receiver BEFORE rclpy.init; it owns no ROS resources.
# Keep the two-second timeout, but do not let a busy ROS callback starve reads.
heartbeat_context=multiprocessing.get_context('fork')
heartbeat=SharedHeartbeat(heartbeat_context,{'last':time.monotonic(),'stop':False,'ready':False,'operator_live':False})
observations={'scan':0,'map':None,'goals':[],'battery':None,'battery_received':0}
listener_done=heartbeat_context.Event()
heartbeat_descriptor=os.dup(sys.stdin.fileno())
def emit(**row):
    row['unix']=time.time()
    try:print(json.dumps(row),flush=True)
    except (BrokenPipeError,OSError):heartbeat['stop']=True
def listen():
    pending=b''
    descriptor=globals().get('heartbeat_descriptor',sys.stdin.fileno())
    while not listener_done.is_set():
        if not select.select([descriptor],[],[],.2)[0]:continue
        chunk=os.read(descriptor,4096)
        if not chunk:heartbeat['stop']=True;return
        pending+=chunk
        if len(pending)>4096:heartbeat['stop']=True;return
        while b'\n' in pending:
            line,pending=pending.split(b'\n',1)
            try:
                msg=json.loads(line.decode('utf-8'))
                if msg.get('cancel'):heartbeat['stop']=True
                if msg.get('heartbeat'):
                    heartbeat['last']=time.monotonic();heartbeat['ready']=True;heartbeat['operator_live']=msg.get('operator_live',True)
            except (ValueError,AttributeError):heartbeat['stop']=True
heartbeat_thread=heartbeat_context.Process(target=listen)
heartbeat_thread.daemon=True
heartbeat_thread.start()
rclpy.init(args=[])
for sig in (signal.SIGHUP,signal.SIGTERM):signal.signal(sig,lambda *_:heartbeat.update(stop=True))
node=rclpy.create_node('web_stock_navigation_session',enable_rosout=False,start_parameter_services=False)
own_uuid=None
class AlreadyAtGoal(Exception):pass
class LocalizationOnlyComplete(Exception):pass
buffer=Buffer();listener=TransformListener(buffer,node);client=None;handle=None
qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
sensor_qos=QoSProfile(depth=1);sensor_qos.reliability=ReliabilityPolicy.BEST_EFFORT
def navigation_scan_receive(message):
    stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
    age=time.time()-stamp
    if not 0<=age<1:return
    observations.update(scan=time.monotonic()-age,scan_message=message)

def stop_odom_receive(message):
    stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
    v=message.twist.twist
    speed=math.sqrt(v.linear.x**2+v.linear.y**2+v.linear.z**2)
    spin=math.sqrt(v.angular.x**2+v.angular.y**2+v.angular.z**2)
    now=time.monotonic()
    if not 0<=time.time()-stamp<.5 or not math.isfinite(speed+spin) or speed>.02 or spin>.03:
        observations['stationary_since']=None;return
    if now-observations.get('stop_odom_at',0)>.5:observations['stationary_since']=None
    if observations.get('stationary_since') is None:observations['stationary_since']=now
    observations['stop_odom_at']=now

def navigation_map_receive(message):
    # Both mapping and label nodes publish /map. A mapping-frame sample
    # arriving last must not erase the navigation-frame retained map.
    if message.header.frame_id=='map':observations['map']=message

subscriptions=[node.create_subscription(LaserScan,NS+REQUEST.get('scan_topic','/scan'),navigation_scan_receive,sensor_qos),
               node.create_subscription(Odometry,NS+'/odom_out',stop_odom_receive,sensor_qos),
               node.create_subscription(MotionStatus,NS+'/motion_status',lambda m:observations.update(motion_status=m,motion_received=time.monotonic()),sensor_qos),
               node.create_subscription(BmsStatus,NS+'/bms_status',lambda m:observations.update(battery=m,battery_received=time.monotonic()),sensor_qos),
               node.create_subscription(OccupancyGrid,NS+'/map',navigation_map_receive,qos),
               node.create_subscription(GoalStatusArray,NS+'/start_algo_task/_action/status',lambda m: observations.update(goals=m.status_list),qos)]
def call(service,kind,request):
    c=node.create_client(kind,NS+'/'+service)
    try:
        if not c.wait_for_service(timeout_sec=3):raise RuntimeError(service+' unavailable')
        f=c.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=200 if service=='stop_algo_task' and request.task_id==0 else 15)
        if not f.done() or f.result() is None:raise RuntimeError(service+' timeout; no retry')
        return f.result()
    finally:node.destroy_client(c)

def ensure_localization_scan():
    if REQUEST.get('navigation_sensor')!='lidar':return
    deadline=time.monotonic()+2.0
    while time.monotonic()<deadline:
        operator_ready()
        rclpy.spin_once(node,timeout_sec=.1)
        if time.monotonic()-observations['scan']<1.0:return
    request=SensorOperation.Request()
    request.sensor_id=request.ID_LIDAR
    request.operation=request.OPR_START
    response=call('sensor_operation',SensorOperation,request)
    emit(event='LIDAR_START_REQUEST',success=bool(response.success),code=int(response.code))
    # Firmware can report success=False/code=0 even after frames resume.
    deadline=time.monotonic()+7.0
    while time.monotonic()<deadline:
        operator_ready()
        rclpy.spin_once(node,timeout_sec=.1)
        if time.monotonic()-observations['scan']<1.0:
            emit(event='LIDAR_SCAN_RECOVERED')
            return
    raise RuntimeError('雷达没有恢复扫描帧；本次不发送定位或导航目标')

def operator_ready():
    if heartbeat['stop'] or not heartbeat.get('ready',True) or not heartbeat.get('operator_live',True) or time.monotonic()-heartbeat['last']>2:
        raise RuntimeError('Preparation cancelled or operator connection lost; no navigation sent')

def battery_ready():
    battery=observations.get('battery')
    if battery is None or time.monotonic()-observations['battery_received']>3:
        raise RuntimeError('Battery feedback expired; no navigation sent')
    if battery.power_wired_charging or battery.power_wp_charging:
        raise RuntimeError('Robot charging; no navigation sent')
    if not 0<=battery.batt_soc<=100:raise RuntimeError('Invalid battery feedback; no navigation sent')

def ensure_navigation_map():
    """Explicitly request the selected native map for every new client.

    The browser may display a saved-file fallback. That is not proof a new
    DDS subscriber received /map. get_label loads and republishes the stock
    map, with no localization reset, motion goal, or map-cell alteration.
    """
    operator_ready()
    assert_active_map('/home/mi/mapping',REQUEST.get('map_name',''))
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise RuntimeError('Task not localized before map refresh')
    request=GetMapLabel.Request();request.map_name=REQUEST.get('map_name','')
    response=call('get_label',GetMapLabel,request)
    if response.success!=0 or response.label.map_name!=request.map_name:
        raise RuntimeError('Native map load failed or selected map differs; no navigation sent')
    grid=response.label.map
    if grid.info.width<=0 or grid.info.height<=0 or grid.info.resolution<=0 or len(grid.data)!=grid.info.width*grid.info.height:
        raise RuntimeError('Native map response incomplete; no navigation sent')
    # Installed GetMapLabel copies metadata/data, not the header. It is the
    # verified native indoor map response, not a synthesized saved-file grid.
    if grid.header.frame_id not in ('','map'):
        raise RuntimeError('Native map frame mismatch; no navigation sent')
    if response.label.is_outdoor:raise RuntimeError('Outdoor map not supported')
    grid.header.frame_id='map'
    observations['map']=grid
    emit(event='NAVIGATION_MAP_LOADED',map_name=request.map_name,width=grid.info.width,height=grid.info.height)

def wait_for_navigation_inputs(localization_only,localization_started):
    """Wait for actual inputs, not an unconditional four-second delay."""
    started=time.monotonic()
    missing=[]
    while time.monotonic()-started<6:
        rclpy.spin_once(node,timeout_sec=.05);operator_ready()
        missing=[]
        try:
            t=buffer.lookup_transform('map','base_link',rclpy.time.Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if abs(time.time()-stamp)>2 or stamp<=localization_started:missing.append('localization TF stale')
        except Exception:missing.append('localization TF missing')
        if not localization_only:
            grid=observations.get('map')
            if grid is None or grid.header.frame_id!='map':missing.append('navigation map missing')
            if time.monotonic()-observations['scan']>1:missing.append('scan stale')
            if observations.get('battery') is None or time.monotonic()-observations['battery_received']>3:missing.append('battery feedback stale')
        if missing:continue
        emit(event='INPUTS_READY',wait_seconds=time.monotonic()-started)
        return
    emit(event='INPUTS_NOT_READY',missing=missing)
    raise RuntimeError('Navigation inputs not ready: '+', '.join(missing)+'; no navigation sent')

def wait_for_stable_localization():
    """Do not accept the first (possibly identity) TF after initialization."""
    started=time.monotonic();baseline=None;since=None;last_stamp=None
    while time.monotonic()-started<10:
        rclpy.spin_once(node,timeout_sec=.1);operator_ready()
        try:t=buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except Exception:
            baseline=None;since=None;continue
        stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
        if abs(time.time()-stamp)>.5:
            baseline=None;since=None;continue
        if stamp==last_stamp:continue
        last_stamp=stamp;p,q=t.transform.translation,t.transform.rotation
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or norm<1e-12:continue
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm)
        if not all(math.isfinite(v) for v in (p.x,p.y,yaw)):continue
        if baseline is None or math.hypot(p.x-baseline[0],p.y-baseline[1])>.1 or abs(math.atan2(math.sin(yaw-baseline[2]),math.cos(yaw-baseline[2])))>math.radians(5):
            baseline=(p.x,p.y,yaw);since=time.monotonic()
        if time.monotonic()-since>=2:
            emit(event='LOCALIZATION_STABLE',wait_seconds=time.monotonic()-started)
            return t
    raise RuntimeError('Localization did not settle; no motion sent')

def check_localization_jump(expected,x,y,yaw):
    if expected is None:return
    if expected.get('frame')!='map' or not all(isinstance(expected.get(k),(int,float)) and math.isfinite(expected[k]) for k in ('x','y','yaw')):
        raise RuntimeError('Invalid reference pose; no navigation sent')
    shift=math.hypot(x-expected['x'],y-expected['y'])
    turn=abs(math.atan2(math.sin(yaw-expected['yaw']),math.cos(yaw-expected['yaw'])))
    if shift>.35 or turn>math.radians(20):
        raise RuntimeError('Localization jumped while robot should be stationary; no navigation sent')

# Map-click goals specify position, not final heading. Keep the stock obstacle
# and oscillation critics; allow a true zero-yaw sample instead of a forced weave.
NAVIGATION_EFFICIENCY_PARAMETERS={
    'FollowPath.max_vel_x':.65, 'FollowPath.max_speed_xy':.65,
    'FollowPath.acc_lim_x':1.3, 'FollowPath.decel_lim_x':-1.,
    'FollowPath.max_vel_theta':.8,
    'FollowPath.acc_lim_theta':1., 'FollowPath.decel_lim_theta':-1.,
    'FollowPath.min_speed_theta':0., 'FollowPath.vtheta_samples':41,
    'FollowPath.vx_samples':12,
    # GoalAlign attracts the nose toward the goal POSITION, not its final yaw.
    # Disabling it also removes the incentive to finish an initial turn.
    # The path-distance critic still tracks the route. Lower heading pressure
    # avoids alternating left/right corrections on otherwise clear straights.
    'FollowPath.PathAlign.scale':16., 'FollowPath.GoalAlign.scale':12.,
    'FollowPath.GoalDist.scale':36.,
    'FollowPath.PathAlign.forward_point_distance':.35,
    'FollowPath.GoalAlign.forward_point_distance':.35,
    'FollowPath.RotateToGoal.scale':0.,
    'FollowPath.trans_stopped_velocity':.05,
    'FollowPath.debug_trajectory_details':False,
    'general_goal_checker.yaw_goal_tolerance':6.28,
}

def apply_navigation_efficiency_profile():
    """Reload cached DWB settings only while the manager is localized and idle.

    No motion publisher, map edits, footprint shrink or obstacle-critic changes.
    Original parameter messages are retained for rollback on reload failure.
    """
    operator_ready()
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise RuntimeError('Cannot tune an active task')
    prefix='controller_server_ab/'
    # Plugins declare FollowPath parameters during configure. Fresh/replaced
    # controllers have no plugin parameters until this non-motion transition.
    state=call(prefix+'get_state',GetState,GetState.Request()).current_state.id
    if state==1:
        req=ChangeState.Request();req.transition.id=1
        if not call(prefix+'change_state',ChangeState,req).success:
            raise RuntimeError('Controller initial configure failed; no motion sent')
        emit(event='NAVIGATION_CONTROLLER_PREPARED')
    request=GetParameters.Request();request.names=list(NAVIGATION_EFFICIENCY_PARAMETERS)
    original=call(prefix+'get_parameters',GetParameters,request).values
    if len(original)!=len(request.names) or any(v.type==0 for v in original):
        raise RuntimeError('Navigation tuning parameters unavailable; no motion sent')
    desired=[Parameter(name,value=value).to_parameter_msg() for name,value in NAVIGATION_EFFICIENCY_PARAMETERS.items()]
    if all(v==p.value for v,p in zip(original,desired)):
        emit(event='NAVIGATION_PROFILE_REUSED');return
    backup=[Parameter(name=name).to_parameter_msg() for name in request.names]
    for p,v in zip(backup,original):p.value=v
    emit(event='NAVIGATION_PROFILE_BACKUP',parameters={name:(v.double_value if v.type==3 else v.integer_value if v.type==2 else v.bool_value) for name,v in zip(request.names,original)})
    def transition(ident):
        req=ChangeState.Request();req.transition.id=ident
        if not call(prefix+'change_state',ChangeState,req).success:
            raise RuntimeError('Navigation controller reload failed')
    def write(parameters):
        req=SetParametersAtomically.Request();req.parameters=parameters
        result=call(prefix+'set_parameters_atomically',SetParametersAtomically,req).result
        if not result.successful:raise RuntimeError('Navigation tuning rejected: '+result.reason)
    state=call(prefix+'get_state',GetState,GetState.Request()).current_state.id
    # DWB critic scales are cached at configure time. A successful parameter
    # service alone is not proof that the controller uses the new values.
    if state==3:transition(4);state=2
    if state==2:transition(2);state=1
    if state!=1:raise RuntimeError('Controller not idle for tuning')
    try:
        operator_ready()
        status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
        if status.status!=8 or status.code!=0:raise RuntimeError('Task changed during tuning')
        write(desired)
        if call(prefix+'get_parameters',GetParameters,request).values!=[p.value for p in desired]:
            raise RuntimeError('Navigation tuning readback mismatch')
        transition(1)  # Leave inactive; the stock Navigation task activates it.
        emit(event='NAVIGATION_PROFILE_APPLIED',parameters=NAVIGATION_EFFICIENCY_PARAMETERS)
    except Exception:
        write(backup)
        state=call(prefix+'get_state',GetState,GetState.Request()).current_state.id
        if state==1:transition(1)
        emit(event='NAVIGATION_PROFILE_ROLLED_BACK')
        raise

def refresh_localization():
    """A new official localization attempt for this goal, not cached status8."""
    emit(event='RELOCALIZATION_BEGIN')
    operator_ready()
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.code!=0 or status.status not in (8,101,6):
        raise RuntimeError('Another task owns the robot; no reset or navigation sent')
    assert_active_map('/home/mi/mapping',REQUEST.get('map_name','room_navigation_20261007'))
    if status.status==8:
        req=StopAlgoTask.Request();req.task_id=0
        response=call('stop_algo_task',StopAlgoTask,req)
        emit(event='LOCALIZATION_RESET_RESULT',result=response.result,code=response.code)
        if response.result!=0 or response.code!=0:raise RuntimeError('Stock localization reset failed')
    operator_ready()
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.code!=0 or status.status not in (101,6):raise RuntimeError('Stock manager not ready for a new localization attempt')
    if 'depth_camera' in (REQUEST.get('mapping_sensor'),REQUEST.get('navigation_sensor')):
        # Stock manager accepts a new localization directly from failed state
        # 6. StopAlgoTask(task_id=0) explicitly rejects state 6 and only
        # creates a false preparation failure here.
        operator_ready()
        ensure_depth_slam(node,call,emit,navigation_only=REQUEST.get('mapping_sensor')!='depth_camera')
        operator_ready()
    # Ordinary depth capture left active by a probe otherwise competes with
    # camera_align for the same D430I and can trigger USB recovery/timeouts.
    prepare_stock_camera('camera_align',node,NS,operator_ready,emit)
    ensure_localization_scan()
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.code!=0 or status.status not in (101,6):raise RuntimeError('Another task started during camera handoff; no localization sent')
    loc=ActionClient(node,Navigation,NS+'/start_algo_task')
    localization_handle=None
    localization_complete=False
    try:
        if not loc.wait_for_server(timeout_sec=3):raise RuntimeError('Localization action unavailable')
        goal=Navigation.Goal();goal.nav_type=7;goal.outdoor=False;goal.map_name=REQUEST.get('map_name','room_navigation_20261007');goal.poses=[]
        begun=time.time();feedback=[]
        def receive(m):
            feedback.append(m.feedback.feedback_code)
            emit(event='RELOCALIZATION_FEEDBACK',code=m.feedback.feedback_code)
        emit(event='RELOCALIZATION_SEND_ONCE')
        f=loc.send_goal_async(goal,feedback_callback=receive)
        rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        if not f.done() or f.result() is None or not f.result().accepted:
            raise RuntimeError('New localization not acknowledged; no retry or navigation')
        localization_handle=f.result()
        result=localization_handle.get_result_async();started=time.monotonic()
        while not result.done() and time.monotonic()-started<150:
            rclpy.spin_once(node,timeout_sec=.1);operator_ready()
        if result.done():
            reply=result.result()
            emit(event='RELOCALIZATION_RESULT',status=None if reply is None else reply.status,
                 result=None if reply is None else reply.result.result,
                 feedback_success=3108 in feedback)
            # The factory result can arrive just before its final 3108
            # feedback. Give that already accepted goal a brief callback drain;
            # never create or send another localization goal here.
            if reply is not None and reply.status==4 and reply.result.result==0 and 3108 not in feedback:
                until=time.monotonic()+1.
                while 3108 not in feedback and time.monotonic()<until:
                    operator_ready()
                    rclpy.spin_once(node,timeout_sec=.05)
        if not result.done() or result.result() is None or result.result().status!=4 or result.result().result.result!=0 or 3108 not in feedback:
            raise RuntimeError('New localization did not succeed; no navigation sent')
        status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
        # The successful action result can precede the manager's status8 update.
        # Wait for that update only; never resubmit localization/navigation.
        deadline=time.monotonic()+5
        while status.status==7 and status.code==0 and time.monotonic()<deadline:
            operator_ready()
            rclpy.spin_once(node,timeout_sec=.1)
            status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
        if status.status!=8 or status.code!=0:raise RuntimeError('New localization state not confirmed')
        emit(event='RELOCALIZATION_SUCCESS',started_unix=begun)
        localization_complete=True
        return begun
    finally:
        # Stop only this accepted localization goal on timeout/operator loss.
        if localization_handle is not None and not localization_complete:
            try:
                cancellation=localization_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node,cancellation,timeout_sec=5)
                response=cancellation.result() if cancellation.done() else None
                emit(event='LOCALIZATION_CANCEL',confirmed=bool(response and response.goals_canceling))
            except Exception as exc:
                emit(event='LOCALIZATION_CANCEL_UNCONFIRMED',message=type(exc).__name__)
        loc.destroy()

def reuse_active_localization():
    """Continuous localization is not reset for each map click."""
    operator_ready()
    assert_active_map('/home/mi/mapping',REQUEST.get('map_name','room_navigation_20261007'))
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise RuntimeError('Active localization required; use webpage localization first')
    deadline=time.monotonic()+3
    while True:
        rclpy.spin_once(node,timeout_sec=.1);operator_ready()
        try:
            t=buffer.lookup_transform('map','base_link',rclpy.time.Time());break
        except Exception:
            if time.monotonic()>=deadline:raise RuntimeError('Active localization TF unavailable; no motion sent')
    stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
    if abs(time.time()-stamp)>2:raise RuntimeError('Active localization TF expired; no motion sent')
    p,q=t.transform.translation,t.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    expected=REQUEST.get('expected_start')
    if expected is None:raise RuntimeError('No verified preview start pose')
    check_localization_jump(expected,p.x,p.y,yaw)
    emit(event='LOCALIZATION_REUSED',x=p.x,y=p.y,yaw=yaw,frame='map')
    return stamp-.001

def replan_from_current_position():
    """Activate only stock planner after reset, and compute a new path."""
    operator_ready();emit(event='REPLANNING_BEGIN')
    state=call('planner_server_ab/get_state',GetState,GetState.Request()).current_state.id
    if state==1:
        req=ChangeState.Request();req.transition.id=1
        if not call('planner_server_ab/change_state',ChangeState,req).success:raise RuntimeError('Planner configure failed')
        state=call('planner_server_ab/get_state',GetState,GetState.Request()).current_state.id
    if state==2:
        req=ChangeState.Request();req.transition.id=3
        if not call('planner_server_ab/change_state',ChangeState,req).success:raise RuntimeError('Planner activation failed')
        state=call('planner_server_ab/get_state',GetState,GetState.Request()).current_state.id
    if state!=3:raise RuntimeError('Stock planner not active')
    planner=ActionClient(node,ComputePathToPose,NS+'/compute_path_to_pose')
    try:
        if not planner.wait_for_server(timeout_sec=4):raise RuntimeError('Planner action unavailable')
        goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=False
        goal.goal.header.frame_id='map';goal.goal.header.stamp=node.get_clock().now().to_msg()
        goal.goal.pose.position.x=REQUEST['x'];goal.goal.pose.position.y=REQUEST['y'];goal.goal.pose.orientation.w=1.
        f=planner.send_goal_async(goal);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        if not f.done() or f.result() is None or not f.result().accepted:raise RuntimeError('Replan not accepted')
        handle=f.result();result=handle.get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=10)
        operator_ready()
        if not result.done():handle.cancel_goal_async();raise RuntimeError('Replan timed out; no navigation sent')
        response=result.result();path=response.result.path
        if response.status!=4 or path.header.frame_id!='map' or not path.poses:raise RuntimeError('No new stock path; no navigation sent')
        points=[{'x':p.pose.position.x,'y':p.pose.position.y} for p in path.poses]
        if not all(math.isfinite(p[k]) for p in points for k in ('x','y')):raise RuntimeError('Invalid replan coordinates')
        if math.hypot(points[-1]['x']-REQUEST['x'],points[-1]['y']-REQUEST['y'])>.25:raise RuntimeError('Replan endpoint outside goal range')
        current=buffer.lookup_transform('map','base_link',rclpy.time.Time())
        if abs(time.time()-current.header.stamp.sec-current.header.stamp.nanosec/1e9)>2:raise RuntimeError('Localization expired during replan')
        if math.hypot(points[0]['x']-current.transform.translation.x,points[0]['y']-current.transform.translation.y)>.5:raise RuntimeError('Replan start mismatch')
        emit(event='REPLAN_SUCCESS',frame='map',points=points)
        length=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:]))
        observations['navigation_timeout']=max(180.,min(3600.,length/.15+120.))
        observations['planned_points']=points
    finally:planner.destroy()

def align_initial_heading():
    """Bounded zero-translation pre-turn while native navigation is still idle.

    The installed robot has no Nav2 Spin server. Reuse its verified manual
    servo template, then END that ownership before submitting native navigation.
    Never overwrite commands from another controller or spin without fresh TF.
    """
    if 'initial_heading' not in globals():
        raise RuntimeError('Web service is still the old version; restart the PC web service; no motion sent')
    t=buffer.lookup_transform('map','base_link',rclpy.time.Time())
    p,q=t.transform.translation,t.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    target=initial_heading(observations.get('planned_points',[]),p.x,p.y,yaw)
    if abs(heading_error(target,yaw))<math.radians(55):return
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise RuntimeError('Task changed before pre-turn')
    publisher=node.create_publisher(MotionServoCmd,NS+'/motion_servo_cmd',10)
    sent=[];conflict=[False]
    def fingerprint(m):return (m.motion_id,m.cmd_source,m.cmd_type,tuple(m.vel_des))
    def observed(m):
        if fingerprint(m) not in sent:conflict[0]=True
    subscription=node.create_subscription(MotionServoCmd,NS+'/motion_servo_cmd',observed,10)
    def send(angular,ending=False):
        m=MotionServoCmd();m.motion_id=303;m.cmd_source=MotionServoCmd.APP
        m.cmd_type=MotionServoCmd.SERVO_END if ending else MotionServoCmd.SERVO_DATA
        m.vel_des=[0.,0.,angular];m.pos_des=[0.,0.,0. if ending else .3]
        m.step_height=[0.,0.] if ending else [.06,.06]
        sent.append(fingerprint(m));del sent[:-100];publisher.publish(m)
    started=time.monotonic();baseline=yaw;progress_at=started;commanded=False
    emit(event='PRETURN_BEGIN',heading=target)
    try:
        while time.monotonic()-started<20:
            rclpy.spin_once(node,timeout_sec=.05);operator_ready();battery_ready()
            if conflict[0]:raise RuntimeError('Other motion controller detected during pre-turn')
            m=observations.get('motion_status')
            if m is None or time.monotonic()-observations.get('motion_received',0)>1:
                raise RuntimeError('Motion status expired during pre-turn')
            # The installed firmware fills unused motor slots with INT32_MIN.
            # Manual control already treats that sentinel as non-fault; the
            # former any(m.motor_error) rejected every normal standing frame.
            if (m.switch_status not in (0,1) or m.ori_error or m.footpos_error or
                    m.motion_id not in (111,112,303) or
                    any(v not in (0,-2147483648) for v in m.motor_error)):
                emit(event='PRETURN_BLOCKED',motion_id=m.motion_id,
                     switch_status=m.switch_status,ori_error=m.ori_error,
                     footpos_error=m.footpos_error,
                     motor_faults=[v for v in m.motor_error if v not in (0,-2147483648)])
                raise RuntimeError('Robot posture/motor not ready for pre-turn')
            if time.monotonic()-observations['scan']>1:raise RuntimeError('Scan expired during pre-turn')
            scan=observations.get('scan_message')
            ranges=[v for v in scan.ranges if math.isfinite(v) and scan.range_min<=v<=scan.range_max] if scan else []
            # Conservative visible clearance for the rotating body. Rear/sides
            # are not fully covered by a forward depth camera: field supervision
            # is still required; this is not a 360-degree collision guarantee.
            if len(ranges)<12:raise RuntimeError('Insufficient lidar data for pre-turn')
            if min(ranges)<.48:
                # A forward-facing lidar cannot certify the whole turn, and
                # its close return may disagree with the depth camera. Leave
                # rotation to stock Nav2 obstacle checking instead of aborting
                # the entire route or forcing an unverified servo turn.
                emit(event='PRETURN_DEFERRED_TO_NAV2',nearest_scan_m=min(ranges))
                return False
            t=buffer.lookup_transform('map','base_link',rclpy.time.Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if abs(time.time()-stamp)>.5:raise RuntimeError('Localization expired during pre-turn')
            p,q=t.transform.translation,t.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            angular=turn_velocity(heading_error(target,yaw))
            if angular==0:
                emit(event='PRETURN_COMPLETE',heading=yaw);return True
            if abs(heading_error(yaw,baseline))>math.radians(5):baseline=yaw;progress_at=time.monotonic()
            if time.monotonic()-progress_at>3:raise RuntimeError('Pre-turn made no measured angular progress')
            if not commanded:
                # START establishes ownership; only DATA carries repeated motion.
                m=MotionServoCmd();m.motion_id=303;m.cmd_source=MotionServoCmd.APP
                m.cmd_type=MotionServoCmd.SERVO_START;m.vel_des=[0.,0.,0.]
                m.pos_des=[0.,0.,.3];m.step_height=[.06,.06]
                sent.append(fingerprint(m));publisher.publish(m)
            send(angular);commanded=True
        raise RuntimeError('Pre-turn timed out')
    finally:
        if commanded and not conflict[0]:
            send(0.,True)
            # Allow the END message to leave DDS before dropping the publisher.
            until=time.monotonic()+.3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_subscription(subscription);node.destroy_publisher(publisher)
def stop_owned():
    started=time.monotonic()
    while own_uuid is not None and time.monotonic()-started<2 and not any(list(s.goal_info.goal_id.uuid)==list(own_uuid.uuid) for s in observations['goals']):
        rclpy.spin_once(node,timeout_sec=0.1)
    owned=own_uuid is not None and any(list(s.goal_info.goal_id.uuid)==list(own_uuid.uuid) and s.status in (1,2,3) for s in observations['goals'])
    if not owned:emit(event='STOP_NOT_SENT',reason='own active goal not confirmed');return
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=1:emit(event='STOP_NOT_SENT',reason='manager no longer navigating');return
    # StopAlgoTask alone may return SUCCESS before the inner Nav2 goal exists.
    # The stock AB reset callback sets is_exit_ first, so startup cannot continue
    # past its CheckExit gate. Keep this restricted to our matched active goal.
    req=SetBool.Request();req.data=True
    response=call('stop_running_robot_navigation',SetBool,req)
    emit(event='STOP_SERVICE_ACK',success=response.success)
    if handle is not None:
        cancellation=handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node,cancellation,timeout_sec=5)
        reply=cancellation.result() if cancellation.done() else None
        emit(event='STOP_ACTION_ACK',accepted=bool(reply and reply.goals_canceling))
    deadline=time.monotonic()+15
    last_idle_check=0.
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        statuses=[s.status for s in observations['goals'] if list(s.goal_info.goal_id.uuid)==list(own_uuid.uuid)]
        if statuses and statuses[-1] in (4,5,6):
            emit(event='STOP_TASK_TERMINAL',status=statuses[-1])
            return
        future=globals().get('result')
        if future is not None and future.done() and future.result() is not None and future.result().status in (4,5,6):
            emit(event='STOP_TASK_TERMINAL',status=future.result().status);return
        # Some factory cancellation paths ACCEPT but omit canceled(result).
        # Confirm the actual idle controller and sustained measured stillness;
        # never treat a cancel acknowledgement alone as a terminal result.
        since=observations.get('stationary_since')
        if (since is not None and time.monotonic()-since>=1.5 and
                time.monotonic()-observations.get('stop_odom_at',0)<.3 and
                time.monotonic()-last_idle_check>=.5):
            last_idle_check=time.monotonic()
            manager=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
            controller=call('controller_server_ab/get_state',GetState,GetState.Request()).current_state.id
            rclpy.spin_once(node,timeout_sec=.05)
            emit(event='STOP_IDLE_CHECK',manager=manager.status,code=manager.code,
                 controller=controller,odom_age=round(time.monotonic()-observations.get('stop_odom_at',0),3))
            if manager.status==101 and manager.code==0 and controller in (1,2) and observations.get('stationary_since') is not None and time.monotonic()-observations['stationary_since']>=1.5 and time.monotonic()-observations.get('stop_odom_at',0)<.3:
                emit(event='STOP_VERIFIED_IDLE',message='Manager idle, controller inactive, odometry stationary; action terminal callback missing')
                return
    emit(event='STOP_UNCONFIRMED',message='No terminal status for owned task; physical stop verification required')
try:
    # The stdin reader starts after ROS imports. Await its first real heartbeat
    # instead of racing its first read and mistaking startup for disconnection.
    waiting=time.monotonic()
    while not (heartbeat['ready'] and heartbeat['operator_live']) and not heartbeat['stop'] and time.monotonic()-waiting<3:
        time.sleep(.02)
    operator_ready()
    localization_only=REQUEST.get('action')=='localize'
    if not localization_only and (REQUEST.get('frame')!='map' or not all(math.isfinite(REQUEST[k]) for k in ('x','y'))):
        raise RuntimeError('Only finite map goals are supported')
    localization_started=refresh_localization() if localization_only else reuse_active_localization()
    if not localization_only:ensure_navigation_map()
    wait_for_navigation_inputs(localization_only,localization_started)
    if localization_only:
        operator_ready()
        t=wait_for_stable_localization()
        stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
        if abs(time.time()-stamp)>2 or stamp<=localization_started:raise RuntimeError('No fresh position after localization')
        p,q=t.transform.translation,t.transform.rotation
        emit(event='POSITION_REFRESHED',x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),frame='map')
        raise LocalizationOnlyComplete()
    if time.monotonic()-observations['scan']>1:raise RuntimeError('No fresh obstacle scan')
    grid=observations['map']
    if grid is None or grid.header.frame_id!='map':raise RuntimeError('No verified map frame')
    q=grid.info.origin.orientation;o=grid.info.origin.position;a=2*math.atan2(q.z,q.w)
    dx=REQUEST['x']-o.x;dy=REQUEST['y']-o.y;r=grid.info.resolution
    ix=math.floor((math.cos(a)*dx+math.sin(a)*dy)/r);iy=math.floor((-math.sin(a)*dx+math.cos(a)*dy)/r)
    if not(0<=ix<grid.info.width and 0<=iy<grid.info.height) or grid.data[iy*grid.info.width+ix]!=0:
        raise RuntimeError('Goal occupied, unknown or outside live map')
    t=buffer.lookup_transform('map','base_link',rclpy.time.Time())
    if abs(time.time()-t.header.stamp.sec-t.header.stamp.nanosec/1e9)>2:raise RuntimeError('Stale localization TF')
    if t.header.stamp.sec+t.header.stamp.nanosec/1e9<=localization_started:raise RuntimeError('Only old localization TF is available')
    p,q=t.transform.translation,t.transform.rotation
    current_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    body_x,body_y=p.x-o.x,p.y-o.y
    body_ix=math.floor((math.cos(a)*body_x+math.sin(a)*body_y)/r)
    body_iy=math.floor((-math.sin(a)*body_x+math.cos(a)*body_y)/r)
    if not(0<=body_ix<grid.info.width and 0<=body_iy<grid.info.height):raise RuntimeError('Robot outside selected map bounds; no navigation sent')
    check_localization_jump(REQUEST.get('expected_start'),p.x,p.y,current_yaw)
    if math.hypot(REQUEST['x']-t.transform.translation.x,REQUEST['y']-t.transform.translation.y)<=.25:
        raise AlreadyAtGoal()
    if 'max_distance_m' in REQUEST:
        limit=REQUEST['max_distance_m']
        if not isinstance(limit,(int,float)) or not math.isfinite(limit) or limit<=0:
            raise RuntimeError('Invalid test distance limit')
        if math.hypot(REQUEST['x']-t.transform.translation.x,REQUEST['y']-t.transform.translation.y)>limit:
            raise RuntimeError('Goal exceeds supervised short-test distance limit')
    operator_ready()
    apply_navigation_efficiency_profile()
    replan_from_current_position()
    # Lifecycle/service work can occupy the executor for several seconds.
    # Re-check current scan/TF after that work, before accepting a motion goal.
    wait_for_navigation_inputs(False,localization_started)
    if align_initial_heading():
        # Rotation changed the start heading; refresh only when actually turned.
        replan_from_current_position()
        wait_for_navigation_inputs(False,localization_started)
    operator_ready()
    battery_ready()
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise RuntimeError('Localization changed before navigation')
    client=ActionClient(node,Navigation,NS+'/start_algo_task')
    if not client.wait_for_server(timeout_sec=3):raise RuntimeError('Navigation unavailable')
    # Service discovery can block callbacks; check again immediately before send.
    wait_for_navigation_inputs(False,localization_started)
    goal=Navigation.Goal();goal.nav_type=1;goal.outdoor=False;goal.map_name=REQUEST.get('map_name','room_navigation_20261007')
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
    p.pose.position.x=REQUEST['x'];p.pose.position.y=REQUEST['y'];p.pose.orientation.w=1.0;goal.poses=[p]
    emit(event='SEND_ONCE',x=REQUEST['x'],y=REQUEST['y'])
    from unique_identifier_msgs.msg import UUID
    own_uuid=UUID();own_uuid.uuid=list(uuid.uuid4().bytes)
    f=client.send_goal_async(goal,goal_uuid=own_uuid,feedback_callback=lambda m:emit(event='FEEDBACK',code=m.feedback.feedback_code,message=m.feedback.feedback_msg))
    rclpy.spin_until_future_complete(node,f,timeout_sec=10)
    if not f.done() or f.result() is None:raise RuntimeError('Goal acknowledgment uncertain; no retry')
    handle=f.result();emit(event='ACK',accepted=handle.accepted)
    if not handle.accepted:raise RuntimeError('Goal rejected')
    result=handle.get_result_async();started=time.monotonic()
    while not result.done():
        rclpy.spin_once(node,timeout_sec=0.1)
        battery_ready()
        reason=('computer_cancel' if heartbeat['stop'] else 'heartbeat_timeout' if time.monotonic()-heartbeat['last']>2 else
                'scan_timeout' if time.monotonic()-observations['scan']>2 else
                'route_timeout' if time.monotonic()-started>observations.get('navigation_timeout',180.) else '')
        if reason:
            emit(event='STOP_REQUESTED',reason=reason);stop_owned();break
    if result.done():emit(event='RESULT',status=result.result().status,result=result.result().result.result)
except LocalizationOnlyComplete:
    pass
except AlreadyAtGoal:
    emit(event='ALREADY_AT_GOAL')
except Exception as exc:
    emit(event='ERROR',message=str(exc))
    if own_uuid is not None:
        try:stop_owned()
        except Exception as stop_exc:emit(event='STOP_ERROR',message=str(stop_exc))
finally:
    try:
        if client is not None:client.destroy()
        node.destroy_node();rclpy.shutdown()
    finally:
        listener_done.set()
        heartbeat_thread.join(timeout=1)
        if heartbeat_thread.is_alive():
            heartbeat_thread.terminate();heartbeat_thread.join(timeout=1)
        os.close(heartbeat_descriptor)
