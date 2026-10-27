"""Robot Python 3.6: ComputePathToPose only, never FollowPath/navigation."""
import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState, ChangeState
from protocol.srv import AlgoTaskStatus,GetMapLabel
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy

NS='/mi_desktop_48_b0_2d_7b_0b_4a'

def service_call(name,kind,request):
    service=node.create_client(kind,NS+'/'+name)
    try:
        if not service.wait_for_service(timeout_sec=3):raise ValueError('规划器服务未就绪')
        future=service.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        if not future.done() or future.result() is None:raise ValueError('规划器准备超时，不发送行走')
        return future.result()
    finally:node.destroy_client(service)

def prepare_planner():
    status=service_call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise ValueError('必须先完成原厂定位')
    state=service_call('planner_server_ab/get_state',GetState,GetState.Request()).current_state.id
    for expected,transition in ((1,1),(2,3)):
        if state==expected:
            request=ChangeState.Request();request.transition.id=transition
            if not service_call('planner_server_ab/change_state',ChangeState,request).success:
                raise ValueError('原厂规划器生命周期切换失败')
            state=service_call('planner_server_ab/get_state',GetState,GetState.Request()).current_state.id
    if state!=3:raise ValueError('原厂规划器未激活')
    ensure_current_costmap()

def matching_geometry(a,b):
    if a is None or b is None:return False
    if a.header.frame_id!=b.header.frame_id or a.info.width!=b.info.width or a.info.height!=b.info.height:return False
    return all(abs(x-y)<1e-5 for x,y in ((a.info.resolution,b.info.resolution),
        (a.info.origin.position.x,b.info.origin.position.x),(a.info.origin.position.y,b.info.origin.position.y),
        (a.info.origin.orientation.z,b.info.origin.orientation.z),(a.info.origin.orientation.w,b.info.origin.orientation.w)))

def wait_current_costmap(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.05)
        if matching_geometry(grids.get('map'),grids.get('costmap')):return True
    return False

def ensure_current_costmap():
    if wait_current_costmap(2):return
    if grids.get('map') is None:raise ValueError('未收到当前地图，不能核对规划器地图')
    status=service_call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status!=8 or status.code!=0:raise ValueError('任务已切换，取消规划器刷新')
    # Recreate the stock static/obstacle layers; never erase cells or shrink
    # inflation to make a route appear. Only while localized, not navigating.
    for transition in (4,2,1,3):
        request=ChangeState.Request();request.transition.id=transition
        if not service_call('planner_server_ab/change_state',ChangeState,request).success:
            raise ValueError('换图后规划器刷新失败，不发送行走')
    grids.pop('costmap',None)
    request=GetMapLabel.Request();request.map_name=REQUEST.get('map_name','')
    loaded=service_call('get_label',GetMapLabel,request)
    if loaded.success!=0 or (request.map_name and loaded.label.map_name!=request.map_name):
        raise ValueError('原厂重新发布的地图与当前选择不一致')
    if not wait_current_costmap(5):raise ValueError('规划器避障地图仍未同步当前地图，请重新定位后重试')
    print(json.dumps({'event':'COSTMAP_REFRESHED','width':grids['map'].info.width,'height':grids['map'].info.height}),flush=True)

rclpy.init(args=[])
node=rclpy.create_node('web_stock_path_preview',enable_rosout=False,start_parameter_services=False)
grids={};qos=QoSProfile(depth=1);qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
subscriptions=[node.create_subscription(OccupancyGrid,NS+'/map',lambda m:grids.update(map=m),qos),
    node.create_subscription(OccupancyGrid,NS+'/global_costmap/costmap',lambda m:grids.update(costmap=m),qos)]
client=ActionClient(node,ComputePathToPose,'/mi_desktop_48_b0_2d_7b_0b_4a/compute_path_to_pose')
try:
    if REQUEST.get('frame')!='map' or not all(math.isfinite(REQUEST[k]) for k in ('x','y')):raise ValueError('目标必须使用map坐标系')
    prepare_planner()
    if not client.wait_for_server(timeout_sec=4):raise ValueError('原厂规划器尚未就绪')
    goal=ComputePathToPose.Goal();goal.goal=PoseStamped();goal.goal.header.frame_id='map';goal.goal.header.stamp=node.get_clock().now().to_msg()
    goal.goal.pose.position.x=REQUEST['x'];goal.goal.pose.position.y=REQUEST['y'];goal.goal.pose.orientation.w=1.0
    goal.planner_id='GridBased';goal.use_start=False
    f=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
    if not f.done() or f.result() is None or not f.result().accepted:raise ValueError('原厂规划请求未接受')
    handle=f.result();f=handle.get_result_async();rclpy.spin_until_future_complete(node,f,timeout_sec=8)
    if not f.done() or f.result() is None:
        handle.cancel_goal_async();raise ValueError('路径计算超时')
    result=f.result();path=result.result.path
    if result.status!=4 or not path.poses:raise ValueError('原厂规划器未找到可行路线')
    points=[{'x':p.pose.position.x,'y':p.pose.position.y} for p in path.poses]
    length=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:]))
    print(json.dumps({'ok':True,'frame':path.header.frame_id,'points':points,'length_m':length,'kind':'stock_nav2_preview'}),flush=True)
except Exception as exc:print(json.dumps({'ok':False,'message':str(exc)}),flush=True)
finally:
    client.destroy();node.destroy_node();rclpy.shutdown()
