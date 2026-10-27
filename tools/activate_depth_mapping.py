"""Run authorized non-motion depth SLAM setup through SSH."""
import subprocess
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
code='NAVIGATION_ONLY='+repr(json.loads((root/'config/scope.json').read_text()).get('mapping_sensor')!='depth_camera')+'\n'+(root/'tools/depth_slam_setup.py').read_text(encoding='utf-8')+'''
import rclpy
from protocol.srv import StopAlgoTask
rclpy.init(args=[])
n=rclpy.create_node('web_depth_slam_setup',enable_rosout=False,start_parameter_services=False)
def emit(**r):print(json.dumps(r),flush=True)
def call(name,kind,request):
    client=n.create_client(kind,NS+'/'+name)
    try:
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Service unavailable: '+name)
        f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=45)
        if not f.done() or f.result() is None:raise RuntimeError('Service response uncertain: '+name)
        return f.result()
    finally:n.destroy_client(client)
try:
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.status==8 and status.code==0:
        request=StopAlgoTask.Request();request.task_id=0
        result=call('stop_algo_task',StopAlgoTask,request)
        emit(event='LOCALIZATION_RESET',result=result.result,code=result.code)
        if result.result!=0 or result.code!=0:raise RuntimeError('Localization reset failed')
    ensure_depth_slam(n,call,emit,allow_mapping=True,navigation_only=NAVIGATION_ONLY)
finally:n.destroy_node();rclpy.shutdown()
'''
result=subprocess.run(['ssh','-F',str(root/'config/readonly_ssh_config'),'-o','BatchMode=yes','cyberdog-current-nx',
    'source /etc/mi/ros2_env.conf; python3 -u -c "import sys;exec(sys.stdin.read())"'],
    input=code,text=True,encoding='utf-8',capture_output=True,timeout=75)
print(result.stdout)
if result.returncode:raise RuntimeError(result.stderr[-1800:])
