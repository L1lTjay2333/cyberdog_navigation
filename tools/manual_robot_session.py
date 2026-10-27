"""Python3.6 robot session. No commands until enabled, armed and healthy."""
import json
import sys
import threading
import time
import copy
import re
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from protocol.msg import MotionServoCmd, MotionStatus, AlgoTaskStatus
from sensor_msgs.msg import LaserScan,Image,CameraInfo,PointCloud2
from front_cloud import forward_cloud_distance
from depth_geometry import depth_xyz,manual_sample_stride
from depth_corridor import observed_corridor
from cloud_geometry import transform_xyz
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time

NS = '/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node = rclpy.create_node('handoff_manual_deadman', enable_rosout=False, start_parameter_services=False)
health = {'sensor_mode':SENSOR_MODE,'auxiliary_task':AUXILIARY_TASK}
depth_info=None
depth_last=-100.
tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
request = {'direction': 'stop', 'armed': False, 'at': -100.}
request_lock = threading.Lock()
running = [True]
active = False
armed_since = None
sent = []
owner_conflict = [False]
safety_latch_reason = ''
sit_future=None;sit_client=None;sit_nonce=None;sit_message=''

def battery(m):
    health['battery_time']=time.monotonic()
    health['battery']={'percent':int(m.batt_soc),'wired_charging':bool(m.power_wired_charging),
        'wireless_charging':bool(m.power_wp_charging),'received_unix':time.time()}
# Process the newest camera frame instead of accumulating large image messages.
camera_qos = copy.copy(qos_profile_sensor_data)
camera_qos.depth = 1
publisher = node.create_publisher(MotionServoCmd, NS + '/motion_servo_cmd', 10)

def fingerprint(m):
    return (m.motion_id, m.cmd_type, m.cmd_source, m.value, tuple(m.vel_des),
            tuple(m.pos_des), tuple(m.step_height))

def observed(m):
    if active and fingerprint(m) not in sent:
        owner_conflict[0] = True

def status(m):
    health['status_time'] = time.monotonic()
    health['status'] = {k: getattr(m, k) for k in
                        ('motion_id', 'switch_status', 'ori_error', 'footpos_error')}
    health['status']['motor_error'] = list(m.motor_error)

def task(m):
    health['task_time'] = time.monotonic()
    health['task'] = int(m.task_status)

def scan(m):
    health['scan_time'] = time.monotonic()
    front, left, right = [], [], []
    for i, v in enumerate(m.ranges):
        if not math.isfinite(v) or not m.range_min <= v <= m.range_max:
            continue
        a = math.atan2(math.sin(m.angle_min+i*m.angle_increment), math.cos(m.angle_min+i*m.angle_increment))
        if abs(a) < math.radians(25): front.append(float(v))
        if math.radians(25) <= a <= math.radians(90): left.append(float(v))
        if math.radians(-90) <= a <= math.radians(-25): right.append(float(v))
    health.update(front_ranges=front, left_ranges=left, right_ranges=right)

def info(m):
    global depth_info
    depth_info=m

def depth(m):
    global depth_last
    now=time.monotonic()
    health['depth_frame_received']=now
    if SENSOR_MODE not in ('depth_forward_trial','supervised_four_way_trial') or now-depth_last<.1:return
    depth_last=now
    if now-health.get('depth_time',-100)>.5:
        health['depth_valid']=False;health['depth_min']=None
    health['depth_invalid_reason']='深度检查未完成'
    try:
        stamp=Time.from_msg(m.header.stamp)
        age=(node.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
        if stamp.nanoseconds<=0 or not -.05<=age<=.35:
            health['depth_invalid_reason']='深度帧时间无效或过期';return
        if depth_info is None:
            health['depth_invalid_reason']='相机内参尚未收到';return
        dt=abs(Time.from_msg(depth_info.header.stamp).nanoseconds-stamp.nanoseconds)*1e-9
        if dt>.2:
            health['depth_invalid_reason']='深度帧与相机内参时间不匹配';return
        points,total=depth_xyz(m,depth_info,stride=manual_sample_stride(m.width,m.height))
        fraction=len(points)/float(total) if total else 0
        t=tf_buffer.lookup_transform('base_link',m.header.frame_id,stamp).transform
        points=transform_xyz(points,[t.translation.x,t.translation.y,t.translation.z],
                             [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
        from front_cloud import forward_points_distance
        try:
            minimum,count=forward_points_distance(points,total,fraction,
                [t.translation.x,t.translation.y,t.translation.z])
        except ValueError as exc:
            health['depth_invalid_reason']=str(exc);return
        health['depth_points_base']=[p for p in points if .1<=p[0]<=4 and abs(p[1])<=2 and .10<=p[2]<=1.5][::2]
        health['depth_stamp_ns']=stamp.nanoseconds
        health.update(depth_time=now-age,depth_valid=True,depth_min=float(minimum),depth_roi_points=count,depth_invalid_reason='',depth_source='depth_image')
    except Exception as exc:
        if now-health.get('depth_time',-100)>.5:health['depth_valid']=False
        health['depth_invalid_reason']='深度投影或坐标变换失败: '+type(exc).__name__

def native_cloud(m):
    now=time.monotonic()
    health['depth_frame_received']=now
    if SENSOR_MODE not in ('depth_forward_trial','supervised_four_way_trial'):return
    # Prefer verified image data when it is already fresh; otherwise use the
    # installed driver's native cloud without requiring CameraInfo/image topics.
    if health.get('depth_valid') and now-health.get('depth_time',-100)<.1:return
    try:
        stamp=Time.from_msg(m.header.stamp)
        age=(node.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
        if stamp.nanoseconds<=0 or not -.05<=age<=.35:raise ValueError('点云帧过期')
        if 'optical' not in m.header.frame_id:raise ValueError('点云不是已验证的光学坐标系')
        t=tf_buffer.lookup_transform('base_link',m.header.frame_id,stamp).transform
        minimum,count=forward_cloud_distance(m,[t.translation.x,t.translation.y,t.translation.z],
            [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
        health.update(depth_time=now-age,depth_valid=True,depth_min=minimum,
            depth_roi_points=count,depth_invalid_reason='',depth_source='native_pointcloud')
    except Exception as exc:
        # An invalid alternate frame must not erase a still-fresh valid image.
        if now-health.get('depth_time',-100)>.5:
            health.update(depth_valid=False,depth_invalid_reason='点云检查失败: '+str(exc))

# Sensor callbacks retain only the newest image/cloud. Projection must not
# block status/odom reception or the command/deadman loop on the ROS thread.
depth_pending={};depth_pending_lock=threading.Lock()
def queue_depth_image(m):
    with depth_pending_lock:depth_pending['image']=m
def queue_native_cloud(m):
    with depth_pending_lock:depth_pending['cloud']=m
def process_depth_pending():
    while rclpy.ok() and running[0]:
        with depth_pending_lock:
            image=depth_pending.pop('image',None);cloud=depth_pending.pop('cloud',None)
        if image is None and cloud is None:time.sleep(.005);continue
        # Native cloud is already projected by the camera driver and is much
        # cheaper to check. Decoding an image first delayed both sources and
        # caused a false stale-frame stop despite fresh cloud frames.
        if cloud is not None:native_cloud(cloud)
        if image is not None and time.monotonic()-health.get('depth_time',-100)>.1:
            depth(image)

subscriptions = [node.create_subscription(MotionStatus, NS+'/motion_status', status, 10),
                 node.create_subscription(AlgoTaskStatus, NS+'/algo_task_status', task, 10),
                 node.create_subscription(LaserScan, NS+'/scan', scan, qos_profile_sensor_data),
                 node.create_subscription(CameraInfo,NS+'/camera/depth/camera_info',info,camera_qos),
                 node.create_subscription(Image,NS+'/camera/depth/image_rect_raw',queue_depth_image,camera_qos),
                 node.create_subscription(PointCloud2,NS+'/camera/depth/color/points',queue_native_cloud,camera_qos),
                 node.create_subscription(MotionServoCmd, NS+'/motion_servo_cmd', observed, 10)]
try:
    from protocol.msg import BmsStatus
    subscriptions.append(node.create_subscription(BmsStatus,NS+'/bms_status',battery,qos_profile_sensor_data))
except ImportError:pass

def reader():
    try:
        for line in sys.stdin:
            try:
                item = json.loads(line)
                if item.get('direction') not in DIRECTIONS: continue
                axes=item.get('axes');command_velocity(item['direction'],axes)
                with request_lock:
                    request.update(direction=item['direction'], armed=item.get('armed') is True,
                                   at=time.monotonic(),axes=axes,action=item.get('action'),nonce=item.get('nonce'))
            except (ValueError, TypeError): pass
    finally:
        running[0] = False

def send(kind, vx=0., vy=0., yaw=0.):
    # Template observed from this robot's App; enum values verified on installed firmware.
    m = MotionServoCmd()
    m.motion_id, m.cmd_source, m.cmd_type, m.value = 303, MotionServoCmd.APP, kind, 0
    m.vel_des = [vx, vy, yaw]
    m.pos_des = [0., 0., 0. if kind == MotionServoCmd.SERVO_END else .3]
    m.step_height = [0., 0.] if kind == MotionServoCmd.SERVO_END else [.06, .06]
    sent.append(fingerprint(m)); del sent[:-100]
    publisher.publish(m)

def stop():
    global active
    if active:
        send(MotionServoCmd.SERVO_DATA)
        send(MotionServoCmd.SERVO_END)
        active = False

def ensure_idle_camera():
    """Idle manual control has no stock algorithm to activate the depth camera."""
    from lifecycle_msgs.srv import GetState,ChangeState
    deadline=time.monotonic()+2
    while 'task' not in health and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if health.get('task') not in (101,6):
        health['camera_start_message']='等待待机任务反馈；建图/定位期间不抢占相机'
        return # Mapping/localization owns its camera.
    # Already-active camera can publish before lifecycle services are discovered.
    # Fresh valid frames are sufficient here; avoid unnecessary ownership RPCs.
    until=time.monotonic()+.7
    while time.monotonic()<until:
        if health.get('depth_valid') and time.monotonic()-health.get('depth_time',-100)<.5:
            health['camera_start_message']='已收到有效实时深度，无需重新启动相机'
            return
        rclpy.spin_once(node,timeout_sec=.03)
    def guard():
        if health.get('task') not in (101,6) or time.monotonic()-health.get('task_time',-100)>2:
            raise RuntimeError('任务已改变，不切换相机')
    def camera_call(kind,service,request):
        client=node.create_client(kind,NS+'/camera/camera/'+service)
        try:
            guard()
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('相机服务不可用')
            f=client.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=6)
            if not f.done() or f.result() is None:raise RuntimeError('相机服务超时')
            guard();return f.result()
        finally:node.destroy_client(client)
    try:
        prepare_stock_camera('camera',node,NS,guard,lambda **_:None)
        state=camera_call(GetState,'get_state',GetState.Request()).current_state.id
        if state==1:
            req=ChangeState.Request();req.transition.id=1
            if not camera_call(ChangeState,'change_state',req).success:raise RuntimeError('相机配置失败')
            state=camera_call(GetState,'get_state',GetState.Request()).current_state.id
        if state==2:
            req=ChangeState.Request();req.transition.id=3
            if not camera_call(ChangeState,'change_state',req).success:raise RuntimeError('相机启动失败')
        elif state!=3:raise RuntimeError('相机处于过渡状态')
        health['camera_start_message']='深度相机已激活，等待有效帧与坐标变换'
    except Exception as exc:
        health['depth_invalid_reason']='相机启动未完成: '+str(exc)
        health['camera_start_message']=health['depth_invalid_reason']
        if health.get('depth_valid') and time.monotonic()-health.get('depth_time',-100)<.5:
            health['camera_start_message']='已收到有效深度；生命周期检查未完成，不重复启动相机'
            health['depth_invalid_reason']=''

ensure_idle_camera()
threading.Thread(target=process_depth_pending,daemon=True).start()
threading.Thread(target=reader, daemon=True).start()
last_report = 0.
last_send = 0.
camera_retry_at=time.monotonic()+3
camera_retry_done=False
try:
    while running[0] and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.03)
        now = time.monotonic()
        with request_lock: current = dict(request)
        # Task discovery can arrive AFTER the initial 2 s bootstrap window.
        # One idle-only retry; never block a running movement to configure camera.
        if not camera_retry_done and now>=camera_retry_at and health.get('task') in (101,6) and not active and not current['armed'] and sit_future is None:
            camera_retry_done=True
            if health.get('depth_time',-100)<0:ensure_idle_camera()
            now=time.monotonic()
        direction = current['direction']
        armed = current['armed'] and now-current['at'] <= .35 and MOTION_ENABLED
        armed = armed and sit_future is None and current.get('action') not in ('sit','posture')
        # Continuous valid operator heartbeat, not a permanent two-second lock.
        # A sensor stop requires release then a NEW key press; never auto-resume.
        if not current['armed']:safety_latch_reason=''
        if sit_future is not None and current['armed']:
            safety_latch_reason='坐下动作执行中，请完成后重新按方向键'
        requested_velocity=command_velocity(direction,current.get('axes'))
        velocity=limited_velocity(direction,current.get('axes'),health,now)
        reason = allowed(direction, health, now)
        if active and reason:
            # A sensor/status failure latches this session; never resume by itself.
            safety_latch_reason = safety_latch_reason or reason
        if safety_latch_reason: reason = '保护停车，请松开按键后重新按下: '+safety_latch_reason
        if owner_conflict[0]: reason = '检测到其他控制来源，请退出手机控制并重新连接'
        if not armed or direction == 'stop' or reason:
            stop()
        elif now-last_send >= .05:
            last_send = now
            vx, vy, yaw = velocity
            if not active:
                active = True
                send(MotionServoCmd.SERVO_START)
            send(MotionServoCmd.SERVO_DATA, vx, vy, yaw)
        if current.get('action') in ('sit','posture') and current.get('nonce')!=sit_nonce:
            sit_nonce=current.get('nonce');stop()
            try:
                if not MOTION_ENABLED:raise RuntimeError('姿态运动权限未开放')
                if owner_conflict[0]:raise RuntimeError('其他控制来源冲突，请退出手机控制并重新连接')
                if current.get('action')=='posture':
                    target_id,label,expected_map=posture_target(health,now)
                else:
                    if allowed('backward',health,now):raise RuntimeError('当前姿态或任务不允许坐下')
                    target_id,label,expected_map=143,'坐下',(62,3)
                paths=[Path('/opt/ros2/cyberdog/share/motion_action/preset/motion_id_map.toml'),Path('/opt/ros2/galactic/share/motion_action/preset/motion_id_map.toml'),Path('/opt/ros/galactic/share/motion_action/preset/motion_id_map.toml')]
                pattern=r'motion_id\s*=\s*'+str(target_id)+r'\s+map\s*=\s*\[\s*'+str(expected_map[0])+r'\s*,\s*'+str(expected_map[1])+r'\s*\]'
                verified=any(p.is_file() and re.search(pattern,p.read_text()) for p in paths)
                if not verified:raise RuntimeError('本机'+label+'动作映射尚未核实，不发送姿态指令')
                if sit_future is not None:raise RuntimeError('坐下请求尚未完成')
                from protocol.srv import MotionResultCmd
                sit_client=node.create_client(MotionResultCmd,NS+'/motion_result_cmd')
                if not sit_client.wait_for_service(timeout_sec=.5):raise RuntimeError('坐下服务不可用')
                req=MotionResultCmd.Request();req.motion_id=target_id;req.cmd_source=MotionResultCmd.Request.APP
                req.duration=0;req.value=0
                sit_future=sit_client.call_async(req);sit_message=label+'请求已提交，等待原厂结果'
            except Exception as exc:
                sit_message=str(exc)
                if sit_client is not None and sit_future is None:node.destroy_client(sit_client);sit_client=None
        if sit_future is not None:
            stop()
            if sit_future.done():
                response=sit_future.result();sit_message='姿态服务结果: '+str(response.result)+' code='+str(response.code)+'；以实时运动状态和现场姿态为准'
                node.destroy_client(sit_client);sit_client=None;sit_future=None
        if now-last_report >= .2:
            last_report = now
            print(json.dumps({'connected': True, 'enabled': MOTION_ENABLED,
                'armed': armed, 'active': active, 'direction': direction,
                'requested_velocity': list(requested_velocity),
                'limited_velocity':list(velocity),
                'front_clearance_m':front_clearance(health) if health.get('depth_valid') and now-health.get('depth_time',-100)<=.5 else None,
                'front_stop_clearance_m':FRONT_STOP_CLEARANCE_M,
                'front_extent_m':FRONT_EXTENT_M,
                'battery':health.get('battery'),'sit_message':sit_message,
                'camera_start_message':health.get('camera_start_message',''),
                'depth_age_s':max(0,now-health['depth_time']) if 'depth_time' in health else None,
                'depth_frame_age_s':max(0,now-health['depth_frame_received']) if 'depth_frame_received' in health else None,
                'reason': reason or ('' if MOTION_ENABLED else '运动锁关闭：仅检查按钮及传感器'),
                'front_min': min(health.get('front_ranges', [])) if health.get('front_ranges') else None,
                'depth_min':health.get('depth_min'),'depth_valid':health.get('depth_valid',False),
                'depth_source':health.get('depth_source','depth_image'),
                'depth_invalid_reason':health.get('depth_invalid_reason',''),
                'forward_block_reason':allowed('forward',health,now),
                'direction_blocks':{d:allowed(d,health,now) for d in DIRECTIONS},
                'sensor_mode':SENSOR_MODE,
                'motion_status': health.get('status'), 'task': health.get('task'),
                'unix': time.time()}), flush=True)
finally:
    stop()
    # Drain a few ROS cycles after issuing stop; physical stop still needs field confirmation.
    until = time.monotonic()+.3
    while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.03)
    node.destroy_node(); rclpy.shutdown()
