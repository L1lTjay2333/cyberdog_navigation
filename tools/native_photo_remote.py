"""One stock native still, transient robot Python3.6. No motion or file deletion."""
import base64,json,re,time
from pathlib import Path
import rclpy
from protocol.srv import CameraService
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

rclpy.init(args=[])
node=rclpy.create_node('computer_native_still',enable_rosout=False,start_parameter_services=False)
try:
    client=node.create_client(CameraService,'/mi_desktop_48_b0_2d_7b_0b_4a/camera_service')
    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('原厂拍照服务不可用')
    # Verify actual fresh RGB input before taking a still. Never retry TAKE_PICTURE.
    images=[]
    sub=node.create_subscription(Image,'/mi_desktop_48_b0_2d_7b_0b_4a/image',lambda msg:images.append(msg.header.stamp),qos_profile_sensor_data)
    deadline=time.monotonic()+2
    while not images and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not images:
        prepare=CameraService.Request();prepare.command=9;prepare.width=1280;prepare.height=960;prepare.fps=5
        future=client.call_async(prepare)
        rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        if not future.done() or future.result() is None or future.result().result!=0:
            raise RuntimeError('彩色预览启动未确认；本次未发送拍照请求')
        deadline=time.monotonic()+5
        while not images and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not images:raise RuntimeError('彩色预览没有实际图像；本次未发送拍照请求')
    node.destroy_subscription(sub)
    request=CameraService.Request();request.command=1;request.width=0;request.height=0
    started=time.time();future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=12)
    if not future.done() or future.result() is None:raise RuntimeError('拍照未确认响应，不自动重试')
    response=future.result();finished=time.time()
    if response.result!=0:
        reason='相机当前未处于可拍照状态；请先启动彩色预览，再停稳拍摄' if response.result==5 else '原厂拍照请求被拒绝'
        raise RuntimeError(reason+'（错误码'+str(response.result)+'）'+response.msg)
    name=response.msg.split(',')[0]
    if not re.fullmatch(r'[0-9]{8}_[0-9]{6}(?:_[0-9]+)?\.jpg',name):raise ValueError('Unexpected stock photo filename')
    folder=Path('/home/mi/Camera').resolve();path=folder/name
    if path.is_symlink() or path.resolve().parent!=folder or not 0<path.stat().st_size<=20*1024*1024:
        raise ValueError('Invalid native JPEG file')
    print(json.dumps({'kind':'native_photo','name':name,'data':base64.b64encode(path.read_bytes()).decode('ascii'),
        'request_start_unix':started,'response_unix':finished,'exposure_timestamp_verified':False,
        'depth_registration_verified':False,'motion_requests':0}),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
