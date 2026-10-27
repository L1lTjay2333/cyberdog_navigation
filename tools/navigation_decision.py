"""Computer-side decisions over stock navigation; never sends motion itself."""
import math
import time
from telemetry_time import sensor_age,event_local_time
from route_preview import world_cell

TOLERANCE_M=.25  # Current stock goal checker; no controller parameter write.

def map_revision(grid):
    origin=grid.get('origin') or {}
    return [grid.get(k) for k in ('frame','width','height','resolution','received_unix')]+[origin.get(k) for k in ('x','y','qz','qw')]

def preview_reusable(selection,target,snapshot,now=None):
    now=time.time() if now is None else now
    if not selection or selection.get('goal')!=target or not 0<=now-selection.get('prepared_unix',0)<30:return False
    if selection.get('map_revision')!=map_revision(snapshot.get('map') or {}):return False
    points=(selection.get('preview') or {}).get('points') or []
    pose=snapshot.get('pose') or {}
    if not points or pose.get('frame')!='map':return False
    try:return math.hypot(points[0]['x']-pose['x'],points[0]['y']-pose['y'])<=.15
    except (KeyError,TypeError):return False

def localization_anchor(snapshot):
    """Map-to-odometry anchor from time-matched measurements, not raw yaw."""
    pose=snapshot.get('pose') or {};odom=snapshot.get('odom') or {}
    if pose.get('frame')!='map' or odom.get('frame')!='odom':return None
    if not all(type(p.get(k)) in (int,float) and math.isfinite(p[k])
               for p in (pose,odom) for k in ('x','y','yaw','received_unix')):return None
    if abs(pose['received_unix']-odom['received_unix'])>.15:return None
    yaw=pose['yaw']-odom['yaw'];c,s=math.cos(yaw),math.sin(yaw)
    return (pose['x']-c*odom['x']+s*odom['y'],
            pose['y']-s*odom['x']-c*odom['y'],yaw,pose['received_unix'])

def validate_goal_distance(pose,goal,limit):
    if limit is None:return
    if type(limit) not in (int,float) or not math.isfinite(limit) or limit<=0:
        raise ValueError('导航距离配置无效')
    distance=math.hypot(goal['x']-pose['x'],goal['y']-pose['y'])
    if distance>limit:
        raise ValueError('目标距离 %.2f 米，超出当前单次测试上限 %.1f 米；请选择更近的目标'%(distance,limit))

def motion_problem(snapshot,now=None):
    now=time.time() if now is None else now
    battery=snapshot.get('battery') or {}
    if sensor_age(snapshot,battery.get('received_unix',0),now)>3:return '电池反馈未就绪，不能执行行走'
    if battery.get('wired_charging') or battery.get('wireless_charging'):return '机器狗正在充电；可预览路线，拔掉充电器后才能行走'
    percent=battery.get('percent')
    if type(percent) not in (int,float) or not math.isfinite(percent) or not 0<=percent<=100:
        return '电池电量反馈无效，不能执行行走'
    return None

def localization_problem(snapshot,now=None,executing=False):
    now=time.time() if now is None else now
    if snapshot.get('connection')!='connected' or now-snapshot.get('local_received_unix',0)>3:
        return '机器人连接已中断'
    validation=snapshot.get('localization_validation') or {}
    if validation.get('required'):return validation.get('reason','搬动后需重新定位')
    emitted=snapshot.get('emitted_unix',0)
    task=snapshot.get('task') or {}
    if task.get('status') not in ((8,1) if executing else (8,)) or emitted-task.get('received_unix',0)>2:
        return '原厂定位或任务状态未就绪'
    pose=snapshot.get('pose') or {}
    if pose.get('frame')!='map' or not all(type(pose.get(k)) in (int,float) and math.isfinite(pose[k]) for k in ('x','y','yaw')) or emitted-pose.get('received_unix',0)>2:
        return '地图定位已过期或坐标不一致'
    scan=snapshot.get('scan') or {}
    if emitted-scan.get('received_unix',0)>2 or scan.get('finite_ranges',0)<1:
        return '实时障碍数据未就绪'
    grid=snapshot.get('map') or {}
    if grid.get('display_only'):return '已显示完整保存地图，等待原厂定位加载同一地图'
    if grid.get('frame')!='map':return '原厂地图未就绪'
    try:
        ix,iy=world_cell(grid,pose['x'],pose['y'])
        if not (0<=ix<grid['width'] and 0<=iy<grid['height']):
            return '机器人定位在地图边界外，请检查二维地图与定位图是否配套'
    except (KeyError,ValueError,TypeError,ZeroDivisionError):
        return '地图边界数据无效'
    return None

def validate_preparation(snapshot,goal,route,now=None):
    problem=localization_problem(snapshot,now)
    if problem:raise ValueError(problem)
    if goal.get('frame')!='map' or not all(type(goal.get(k)) in (int,float) and math.isfinite(goal[k]) for k in ('x','y')):
        raise ValueError('目标必须是有效地图坐标')
    grid=snapshot['map']
    if not grid.get('width') or not grid.get('height') or len(grid.get('data',[]))!=grid['width']*grid['height']:
        raise ValueError('地图数据不完整')
    ix,iy=world_cell(grid,goal['x'],goal['y'])
    if not(0<=ix<grid['width'] and 0<=iy<grid['height']) or grid['data'][iy*grid['width']+ix]!=0:
        raise ValueError('目标位于障碍、未知区域或地图外')
    points=route.get('points') or []
    if route.get('frame')!='map' or route.get('kind')!='stock_nav2_preview' or not points:
        raise ValueError('原厂尚未给出可行路线')
    if not all(type(p.get(k)) in (int,float) and math.isfinite(p[k]) for p in points for k in ('x','y')):
        raise ValueError('路线坐标无效')
    if math.hypot(points[-1]['x']-goal['x'],points[-1]['y']-goal['y'])>TOLERANCE_M:
        raise ValueError('路线终点未进入目标范围，请重新选择可达空地')
    pose=snapshot['pose']
    if math.hypot(points[0]['x']-pose['x'],points[0]['y']-pose['y'])>.5:
        raise ValueError('规划起点与实时位置不一致，请重新预览')
    return {'goal':dict(goal),'preview':route,'map_revision':map_revision(grid),'prepared_unix':time.time() if now is None else now}

class NavigationDecision:
    def __init__(self):
        self.session=None;self.best_distance=None;self.last_progress=0;self.started=None;self.anchor=None
    def evaluate(self,snapshot,transport,selection,now=None):
        now=time.time() if now is None else now
        active=transport.get('active',False)
        goal=transport.get('goal') if active else (selection or {}).get('goal')
        session=transport.get('session_started_unix')
        applies=goal==transport.get('goal') and (active or (selection or {}).get('prepared_unix',0)<=(session or 0))
        events=transport.get('events',[]) if applies else []
        # A failed preparation never submitted a motion goal. If its inputs
        # recover, permit an explicit user retry of the still valid selection.
        # Never use this for a submitted/cancelled/uncertain motion task.
        preparation_error=next((e for e in reversed(events) if e.get('event') in ('ERROR','LOCAL_ERROR')),None)
        if (not active and preparation_error and selection and
                not any(e.get('event') in ('SEND_ONCE','STOP_REQUESTED','STOP_UNCONFIRMED') for e in events)):
            if localization_problem(snapshot,now) is None and motion_problem(snapshot,now) is None:
                try:validate_preparation(snapshot,selection['goal'],selection['preview'],now)
                except (ValueError,KeyError,TypeError):pass
                else:events=[]
        if session!=self.session:
            self.session=session;self.best_distance=None;self.last_progress=now;self.started=None;self.anchor=None
        pose=snapshot.get('pose') or {};distance=None
        if goal and pose.get('frame')=='map' and all(type(pose.get(k)) in (int,float) and math.isfinite(pose[k]) for k in ('x','y')):
            distance=math.hypot(pose['x']-goal['x'],pose['y']-goal['y'])
        def report(stage,message,stop=False):
            return {'stage':stage,'message':message,'distance_m':distance,'stop_required':stop,'can_start':stage=='READY' and transport.get('enabled',False),'tolerance_m':TOLERANCE_M}
        result=next((e for e in reversed(events) if e.get('event')=='RESULT'),None)
        stop_requested=any(e.get('event') in ('STOP_REQUESTED','STOP_RESULT','STOP_SERVICE_ACK','STOP_ACTION_ACK','STOP_UNCONFIRMED') for e in events)
        stopped=any(e.get('event') in ('STOP_TASK_TERMINAL','STOP_VERIFIED_IDLE') for e in events) or bool(stop_requested and result and result.get('status') in (4,5,6))
        if stop_requested:
            if any(e.get('event')=='STOP_VERIFIED_IDLE' for e in events):return report('CANCELLED','已核实停车：任务空闲、控制器停用、里程计静止；请重新检查定位')
            if stopped:return report('CANCELLED','导航任务已结束；仍需现场确认狗已停稳')
            return report('STOPPING' if active else 'STOP_UNCONFIRMED','停车尚未确认，请现场急停；禁止继续行走',active)
        if result:
            if result.get('status')==4 and result.get('result')==0:
                if localization_problem(snapshot,now) or distance is None:return report('VERIFY','原厂已完成，等待实时位置核实')
                if distance<=TOLERANCE_M:return report('ARRIVED','已到达目标范围')
                return report('VERIFY','原厂已完成，但实测终点超出目标范围')
            if stopped:return report('CANCELLED','导航已取消，原厂停止接口已确认')
            return report('FAILED','原厂导航未完成；不自动重试')
        if stopped:return report('CANCELLED','原厂停止接口已确认，等待客户端收尾')
        if any(e.get('event')=='POSITION_REFRESHED' for e in events):
            problem=localization_problem(snapshot,now)
            return report('WAITING' if problem else 'LOCALIZED',problem or '重新定位成功，位置和朝向已刷新')
        if any(e.get('event')=='ALREADY_AT_GOAL' for e in events):
            return report('AT_GOAL','本次重新定位确认已在目标范围，未发送行走目标')
        error=next((e for e in reversed(events) if e.get('event') in ('ERROR','LOCAL_ERROR','STOP_ERROR')),None)
        if error:
            return report('FAULT','导航未执行：'+error.get('message','导航接口异常'),active)
        failure=next((e for e in reversed(events) if e.get('event')=='FEEDBACK' and e.get('code') in (301,302,303,304,305,306,311,1002)),None)
        if failure:return report('BLOCKED' if failure['code']==306 else 'FAULT','原厂无法完成路线，停止本次任务',active)
        if active and (transport.get('preparing_localization') or any(e.get('event')=='RELOCALIZATION_BEGIN' for e in events)) and not any(e.get('event')=='SEND_ONCE' for e in events):
            if any(e.get('event')=='RELOCALIZATION_SUCCESS' for e in events):
                if goal is None:return report('REFRESHING','本次定位已成功，正在刷新位置和朝向')
                return report('REPLANNING','本次定位已成功，正在从当前位置重新规划')
            feedback=next((e for e in reversed(events) if e.get('event')=='RELOCALIZATION_FEEDBACK'),None)
            if feedback and feedback.get('code')==3107:
                return report('RELOCALIZING','原厂反馈3107：场景定位未匹配成功，正在重试；网页连接正常时这不是掉线，不会行走')
            if goal is None:return report('RELOCALIZING','原厂正在定位场景；这一步不会行走，请等待定位结果')
            return report('RELOCALIZING','执行前重新定位中，成功后才会行走')
        if active and transport.get('preparing_navigation') and not any(e.get('event')=='SEND_ONCE' for e in events):
            return report('REPLANNING','沿用当前定位，正在核对并规划路线；尚未发送行走目标')
        sent=next((e for e in events if e.get('event')=='SEND_ONCE'),None)
        running=any(e.get('event')=='FEEDBACK' and e.get('code')==307 for e in events)
        if active and sent and not running and 0<=now-event_local_time(snapshot,sent)<10:
            # Manager transitions through idle/preparation after accepting the
            # action. Do not cancel solely because its task/TF is switching.
            if snapshot.get('connection')!='connected' or now-snapshot.get('local_received_unix',0)>3:
                return report('STOPPING','机器人实时反馈中断',True)
            problem=motion_problem(snapshot,now)
            if problem:return report('STOPPING',problem,True)
            return report('STARTING','导航目标已提交，等待原厂启动反馈；不是网页断连')
        problem=localization_problem(snapshot,now,executing=active)
        if problem:return report('STOPPING' if active else 'WAITING',problem,active)
        battery_problem=motion_problem(snapshot,now)
        if battery_problem:return report('STOPPING' if active else 'WAITING',battery_problem,active)
        if active:
            anchor=localization_anchor(snapshot)
            if anchor is not None:
                previous=self.anchor
                if previous is not None and 0<anchor[3]-previous[3]<=2:
                    turn=abs(math.atan2(math.sin(anchor[2]-previous[2]),math.cos(anchor[2]-previous[2])))
                    # Compare at the robot, not the odometry origin: a small
                    # heading correction far from that origin must not amplify.
                    odom=snapshot['odom'];a=previous[2]
                    px=previous[0]+math.cos(a)*odom['x']-math.sin(a)*odom['y']
                    py=previous[1]+math.sin(a)*odom['x']+math.cos(a)*odom['y']
                    shift=math.hypot(pose['x']-px,pose['y']-py)
                    if shift>.35 or turn>math.radians(25):
                        return report('STOPPING','定位相对里程计发生跳变，请停稳后重新定位；不是网页掉线',True)
                self.anchor=anchor
            running=any(e.get('event')=='FEEDBACK' and e.get('code')==307 for e in events)
            if running and self.started is None:self.started=now;self.last_progress=now
            if distance is not None and (self.best_distance is None or distance<self.best_distance-.05):
                self.best_distance=distance;self.last_progress=now
            if self.started is not None and now-self.last_progress>30:
                return report('BLOCKED','长时间没有接近目标，停止本次任务',True)
            return report('EXECUTING' if running else 'STARTING','正在行走，由原厂实时规划绕障' if running else '原厂正在准备导航')
        if not selection:return report('IDLE','定位就绪，请在二维地图选择目标')
        try:validate_preparation(snapshot,selection['goal'],selection['preview'],now)
        except (ValueError,KeyError,TypeError) as exc:return report('RESELECT',str(exc))
        if distance is not None and distance<=TOLERANCE_M:return report('AT_GOAL','当前位置已在目标范围，无需行走')
        message='路线已就绪，可进行现场导航测试' if transport.get('enabled') else '路线已就绪，等待现场演示开放运动'
        if preparation_error and not events:message='上次准备未完成：'+preparation_error.get('message','')+'；当前数据已恢复，可点击执行重试'
        return report('READY',message)
