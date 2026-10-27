"""ROS-independent low-speed exploration trajectory policy (Python 3.6)."""
import math
from manual_policy import allowed, front_clearance
from unknown_goal import world_cell, number

MAX_FORWARD=.15
MAX_YAW=.25
RADIUS=.34

def checked_velocity(values):
    if not isinstance(values,(list,tuple)) or len(values)!=3:
        raise ValueError('自动速度格式无效')
    vx,vy,w=[number(v) for v in values]
    if not 0<=vx<=MAX_FORWARD or vy!=0 or abs(w)>MAX_YAW:
        raise ValueError('自动探索速度越界')
    return vx,vy,w

def sensor_problem(health,now,require_depth=True):
    # Reuse existing posture/charging/motor/task guards, NOT permission to reverse.
    reason=allowed('backward',health,now)
    if reason:return reason
    if health.get('task')!=5:return '自动探索要求原厂建图任务保持运行'
    b=health.get('battery') or {}
    if time_age(health.get('battery_time'),now)>3 or type(b.get('percent')) is not int:
        return '电量反馈过期'
    if not 0<=b['percent']<=100:return '电量反馈无效'
    if time_age(health.get('scan_time'),now)>.5:return '雷达扫描过期'
    if require_depth and (time_age(health.get('depth_time'),now)>.5 or not health.get('depth_valid')):
        return '实时深度未通过：'+health.get('depth_invalid_reason','等待有效深度')
    return ''

def time_age(stamp,now):
    return now-stamp if type(stamp) in (int,float) and math.isfinite(stamp) else float('inf')

def safe_point(grid,x,y,radius=RADIUS):
    cx,cy=world_cell(grid,x,y);w,h=grid['width'],grid['height']
    n=math.ceil(radius/grid['resolution'])
    if cx-n<0 or cy-n<0 or cx+n>=w or cy+n>=h:return False
    return all(grid['data'][y*w+x]==0 for y in range(cy-n,cy+n+1) for x in range(cx-n,cx+n+1))

def trajectory_clear(grid,pose,velocity,horizon=1.):
    vx,_,w=checked_velocity(velocity)
    yaw=number(pose['yaw']);px,py=pose['x'],pose['y']
    for step in range(11):
        t=horizon*step/10.
        if abs(w)<1e-6:x,y=px+vx*t*math.cos(yaw),py+vx*t*math.sin(yaw)
        else:x,y=px+vx/w*(math.sin(yaw+w*t)-math.sin(yaw)),py-vx/w*(math.cos(yaw+w*t)-math.cos(yaw))
        if not safe_point(grid,x,y):return False
    return True

def track(grid,pose,route,health,now):
    reason=sensor_problem(health,now)
    if reason:return (0.,0.,0.),reason
    points=(route or {}).get('points') or []
    if not points or route.get('frame')!=pose.get('frame') or grid['frame']!=pose['frame']:
        return (0.,0.,0.),'轨迹或坐标系未就绪'
    # Follow local axis-aligned path, stop/turn at corners rather than cut them.
    nearest=min(range(len(points)),key=lambda i:(points[i]['x']-pose['x'])**2+(points[i]['y']-pose['y'])**2)
    target=points[-1]
    for p in points[nearest+1:]:
        if math.hypot(p['x']-pose['x'],p['y']-pose['y'])>=.12:
            target=p;break
    dx,dy=target['x']-pose['x'],target['y']-pose['y']
    distance=math.hypot(dx,dy)
    if distance<.04:return (0.,0.,0.),'等待更新观察点'
    error=math.atan2(math.sin(math.atan2(dy,dx)-pose['yaw']),math.cos(math.atan2(dy,dx)-pose['yaw']))
    yaw=max(-MAX_YAW,min(MAX_YAW,error))
    vx=0. if abs(error)>.30 else min(MAX_FORWARD,distance*.7)
    clearance=front_clearance(health)
    if vx>0:
        if clearance is None:return (0.,0.,0.),'前方深度距离无效'
        vx=min(vx,max(0.,clearance-.20))
    velocity=(vx,0.,yaw)
    if not trajectory_clear(grid,pose,velocity):
        return (0.,0.,0.),'当前动态地图不允许本段轨迹，等待重新规划'
    return velocity,''

def overlay_depth(grid,pose,points):
    """Points must already be in the SAME grid frame at acquisition time."""
    result={**grid,'data':list(grid['data'])};w,h=grid['width'],grid['height']
    for p in points:
        if len(p)!=3 or not all(math.isfinite(v) for v in p):continue
        # Ground removed in base_link before transformation; do not clear lidar data.
        x,y=world_cell(grid,p[0],p[1])
        if 0<=x<w and 0<=y<h:result['data'][y*w+x]=100
    return result
