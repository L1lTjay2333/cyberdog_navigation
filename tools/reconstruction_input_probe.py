"""Transient stock-camera readiness probe (robot Python 3.6). No motion calls."""
import json
import math
import time
import numpy as np
import rclpy
from lifecycle_msgs.srv import GetState,ChangeState
from protocol.srv import AlgoTaskStatus
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from reconstruction_core import FrameQueue

NS='/mi_desktop_48_b0_2d_7b_0b_4a'
CONNECT=bool(globals().get('CONNECT_CAMERA',False))
rclpy.init(args=[])
n=rclpy.create_node('reconstruction_input_check',enable_rosout=False,start_parameter_services=False)
buffer=Buffer();listener=TransformListener(buffer,n);queue=FrameQueue()
info=[None];counts={'images':0,'valid_depth':0,'matched_pose':0};changes=[]
last_valid_stamp=[0]
result={'ready':False,'camera_ready':False,'control_requests':0}

def call(path,kind,request):
    client=n.create_client(kind,path)
    try:
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('原厂相机/任务状态接口不可用')
        future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=6)
        if not future.done() or future.result() is None:raise RuntimeError('原厂接口未确认响应；没有自动重试')
        return future.result()
    finally:n.destroy_client(client)

def camera_state(name):return call(NS+'/camera/'+name+'/get_state',GetState,GetState.Request()).current_state.id
def task_state():
    response=call(NS+'/algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    return {'status':int(response.status),'code':int(response.code)}
def guard():
    task=task_state()
    if task!={'status':101,'code':0} or camera_state('camera_align')!=1:
        raise RuntimeError('其他任务已占用相机；未停止或接管手机任务')

def receive(message):
    counts['images']+=1
    calibration=info[0]
    if calibration is None:return
    stamp=Time.from_msg(message.header.stamp)
    if stamp.nanoseconds<=last_valid_stamp[0]:return
    age=(n.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
    if (not -.05<=age<=.7 or message.header.frame_id!=calibration.header.frame_id
            or message.width!=calibration.width or message.height!=calibration.height
            or not 1<=message.width<=1280 or not 1<=message.height<=1024
            or abs(Time.from_msg(calibration.header.stamp).nanoseconds-stamp.nanoseconds)*1e-9>.2
            or message.encoding not in ('16UC1','32FC1')):return
    p=list(calibration.p)
    if (len(p)!=12 or not all(math.isfinite(float(x)) for x in p) or p[0]<=0 or p[5]<=0
            or not 0<=p[2]<message.width or not 0<=p[6]<message.height or p[3]!=0 or p[7]!=0):return
    item=2 if message.encoding=='16UC1' else 4
    if not message.width*item<=message.step<=message.width*item+4096 or len(message.data)!=message.step*message.height:return
    dtype=('>' if message.is_bigendian else '<')+('u2' if item==2 else 'f4')
    depth=np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),strides=(message.step,item))[::16,::16].astype(float)
    if item==2:depth*=.001
    if not np.any(np.isfinite(depth)&(depth>=.15)&(depth<=4.)):return
    last_valid_stamp[0]=stamp.nanoseconds
    counts['valid_depth']+=1
    queue.offer({'stamp':stamp.nanoseconds/1e9,'message':message},time.monotonic())

def lookup(item):
    message=item['message'];stamp=Time.from_msg(message.header.stamp)
    age=(n.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
    if not -.05<=age<=2.:return None
    try:
        transform=buffer.lookup_transform('vodom',message.header.frame_id,stamp)
        p,q=transform.transform.translation,transform.transform.rotation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        tfstamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
        if (not all(math.isfinite(v) for v in values) or abs(tfstamp-item['stamp'])>1e-6
                or not .5<=math.sqrt(sum(v*v for v in values[3:]))<=1.5):return None
        return True
    except Exception:return None

try:
    task=task_state();state=camera_state('camera');other=camera_state('camera_align')
    result.update(task=task,camera_state=state,camera_align_state=other)
    if CONNECT:
        if task=={'status':101,'code':0} and other==1 and state in (1,2,3):
            for transition in ([1,3] if state==1 else [3] if state==2 else []):
                guard();request=ChangeState.Request();request.transition.id=transition
                response=call(NS+'/camera/camera/change_state',ChangeState,request)
                actual=camera_state('camera');changes.append({'transition':transition,'state':actual,'success':bool(response.success)})
                if not response.success or actual!={1:2,3:3}[transition]:raise RuntimeError('相机初始化未成功；未重试')
            result['camera_state']=camera_state('camera')
        elif task!={'status':5,'code':0} or state!=3 or other!=1:
            raise RuntimeError('原厂任务或其他相机正在占用；请先停止手机建图/导航，再连接')
    subs=[n.create_subscription(CameraInfo,NS+'/camera/depth/camera_info',lambda m:info.__setitem__(0,m),qos_profile_sensor_data),
          n.create_subscription(Image,NS+'/camera/depth/image_rect_raw',receive,qos_profile_sensor_data)]
    end=time.monotonic()+6
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        counts['matched_pose']+=len(queue.ready(lookup,time.monotonic()))
        if counts['matched_pose']>=3 and counts['valid_depth']>=3:break
    result.update(camera_ready=counts['valid_depth']>=3,ready=counts['matched_pose']>=3)
    result['task']=task_state()
    if result['ready'] and result['task']['status']!=5:
        result.update(ready=False,message='深度图正常，但原厂扫描位姿任务已停止')
    if result['task']['status'] not in (5,101) or result['task']['code']!=0:
        result['ready']=False
        result['message']='原厂任务状态变化；请保持手机不启停建图/导航并重新检查'
    elif result['ready']:result['message']='深度图与同时间位姿已就绪'
    elif not result['camera_ready']:
        result['message']=('深度相机未开启，请点击连接/检查深度相机' if result.get('camera_state')!=3
                           else '相机报告已启用，但未收到足够的新鲜有效深度帧，请检查设备输入')
    else:result['message']='深度相机正常，但缺少同时间位姿；请连接相机以准备扫描输入'
except Exception as error:result.update(ready=False,message=str(error))
finally:
    result.update(counts=counts,changes=changes)
    print(json.dumps(result,allow_nan=False),flush=True)
    n.destroy_node();rclpy.shutdown()
