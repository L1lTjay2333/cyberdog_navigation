"""Map-independent, low-speed manual control policy (also used on robot)."""
import math

ARC_YAW_RATE = .60 # rad/s at full steering; staged increase from .30/.45.
FRONT_EXTENT_M = .25 # Existing configured footprint front, relative to base_link.
FRONT_STOP_CLEARANCE_M = .20
APPROACH_TIME_S = 1.0 # Approach speed budget; physical braking still needs testing.

def front_clearance(health):
    closest=health.get('depth_min')
    if type(closest) not in (int,float) or not math.isfinite(closest):return None
    return closest-FRONT_EXTENT_M

DIRECTIONS = {'forward': (.5, 0., 0.), 'backward': (-.5, 0., 0.),
              'shift_left': (0., .3, 0.), 'shift_right': (0., -.3, 0.),
              'left': (0., 0., ARC_YAW_RATE), 'right': (0., 0., -ARC_YAW_RATE), 'stop': (0., 0., 0.)}
for name,x,y in (('forward_left',1,1),('forward_right',1,-1),('backward_left',-1,1),('backward_right',-1,-1)):
    # Steering toward travel-left/right: reverse travel reverses body yaw.
    DIRECTIONS[name]=(.5*x/math.sqrt(2),0.,ARC_YAW_RATE*x*y/math.sqrt(2))

def checked_axes(axes):
    if not isinstance(axes,(list,tuple)) or len(axes)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in axes):
        raise ValueError('摇杆向量无效')
    if sum(v*v for v in axes)>1.000001:raise ValueError('摇杆向量越界')
    return list(axes)

def axes_direction(axes):
    x,y=checked_axes(axes)
    if abs(x)<.01:x=0
    if abs(y)<.01:y=0
    if x==0:return 'left' if y>0 else 'right' if y<0 else 'stop'
    return ('forward' if x>0 else 'backward')+('_left' if y>0 else '_right' if y<0 else '')

def command_velocity(direction,axes=None):
    if axes is None:return DIRECTIONS[direction]
    x,y=checked_axes(axes)
    if axes_direction(axes)!=direction:raise ValueError('方向与摇杆向量不一致')
    if abs(x)<.01:return (0.,0.,ARC_YAW_RATE*y)
    return (.5*x,0.,ARC_YAW_RATE*y*(1 if x>0 else -1))

def limited_velocity(direction,axes,health,now):
    velocity=command_velocity(direction,axes)
    if allowed(direction,health,now):return (0.,0.,0.)
    if velocity[0]<=0 or health.get('sensor_mode') not in ('depth_forward_trial','supervised_four_way_trial'):
        return velocity
    clearance=front_clearance(health)
    budget=max(0.,clearance-FRONT_STOP_CLEARANCE_M)/APPROACH_TIME_S
    factor=min(1.,budget/velocity[0])
    # Scale turn with forward speed: preserve the requested arc, don't spin at a stop.
    return tuple(v*factor for v in velocity)

def posture_target(health, now):
    """Toggle only from fresh, recognized stationary poses; no depth prerequisite."""
    if now-health.get('status_time',-100)>1:raise ValueError('运动状态未收到或过期')
    s=health.get('status',{})
    if s.get('switch_status')==14:raise ValueError('机器狗反馈充电状态，请拔掉充电线并等待状态恢复')
    if s.get('switch_status')!=0 or s.get('ori_error',1) or s.get('footpos_error',1):
        raise ValueError('姿态异常或动作切换中，请等待并检查手机状态')
    if any(v not in (0,-2147483648) for v in s.get('motor_error',[])):raise ValueError('电机状态异常')
    b=health.get('battery') or {}
    if b.get('wired_charging') or b.get('wireless_charging'):raise ValueError('充电中不执行姿态动作')
    tasks=(101,6,8)
    if health.get('auxiliary_task') in (5,7):tasks+=(health['auxiliary_task'],)
    if now-health.get('task_time',-100)>2 or health.get('task') not in tasks:
        raise ValueError('任务状态过期或其他任务执行中')
    motion=s.get('motion_id')
    if motion in (111,112):return 101,'趴下',(7,1)
    if motion in (101,102,143):return 111,'起立',(12,0)
    raise ValueError('当前不是已确认的站立或趴下姿态，不自动切换')

def allowed(direction, health, now):
    if direction not in DIRECTIONS:
        return '未知方向'
    if direction == 'stop':
        return ''
    battery=health.get('battery') or {}
    if battery.get('wired_charging') or battery.get('wireless_charging'):return '机器狗正在充电，不发送运动'
    if now - health.get('status_time', -100.) > 1.:
        return '运动状态未收到或过期'
    s = health.get('status', {})
    if s.get('switch_status')==14:return '机器狗反馈充电状态，请拔掉充电线并等待状态恢复'
    if s.get('switch_status') not in (0, 1) or s.get('ori_error', 1) or s.get('footpos_error', 1):
        return '姿态、急停或运动状态异常'
    if any(v not in (0, -2147483648) for v in s.get('motor_error', [])):
        return '存在未解释的电机状态，禁止运动'
    supervised = health.get('sensor_mode') == 'supervised_four_way_trial'
    depth_mode = health.get('sensor_mode') in ('depth_forward_trial','supervised_four_way_trial')
    # Stock enum 6 is localization failed, not an executing navigation task.
    permitted_tasks = (101, 8, 6) if depth_mode else (101, 8)
    if supervised and health.get('auxiliary_task') in (5,7):
        permitted_tasks += (health['auxiliary_task'],)
    if now - health.get('task_time', -100.) > 2. or health.get('task') not in permitted_tasks:
        return '其他算法任务正在运行或任务状态过期'
    if supervised:
        if s.get('motion_id') not in (111,112,303):return '请先让机器狗正常站立，当前姿态未通过'
        if direction in ('backward','shift_left','shift_right','left','right','backward_left','backward_right'):
            return '' # Supervised, short-duration teleoperation; no rear/lateral obstacle avoidance claim.
    if direction == 'backward':
        return '后方无已验证测距覆盖，暂不开放后退'
    if direction in ('shift_left', 'shift_right'):
        return '横移控制与侧后方保护尚未实机验证，暂不开放横移'
    if depth_mode:
        if direction not in (('forward','forward_left','forward_right') if supervised else ('forward',)):return '深度方案当前仅开放前进测试，转向尚未验收'
        if now-health.get('depth_time',-100.)>.5:
            if now-health.get('depth_frame_received',-100.)<=.5 and health.get('depth_invalid_reason'):
                return '深度无效：'+health['depth_invalid_reason']
            return '深度帧中断或过期，等待相机恢复（不是检测到近障碍）'
        if not health.get('depth_valid'):return '深度无效：'+(health.get('depth_invalid_reason') or '覆盖、标定或坐标变换未通过')
        clearance=front_clearance(health)
        if clearance is None:return '前方无有效深度距离'
        if clearance<=FRONT_STOP_CLEARANCE_M+1e-9:return '前沿估算净距≤20厘米，锁定前进'
        return ''
    if now - health.get('scan_time', -100.) > .5:
        return '雷达数据过期'
    values = health.get('front_ranges', [])
    if not values or not all(math.isfinite(v) and v > .6 for v in values):
        return '前方回波过近或缺少有效测距'
    if direction in ('left', 'right'):
        side = health.get(direction + '_ranges', [])
        if not side or min(side) <= .6:
            return '转向侧净空不足或无有效测距'
    return ''
