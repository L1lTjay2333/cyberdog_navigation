"""Computer-side serialized camera preparation. Never sends robot motion."""
import json
import shlex
import subprocess
import time
from pathlib import Path


class InputCheckError(RuntimeError):
    def __init__(self,message,stderr=''):
        super().__init__(message);self.stderr=stderr


def probe(root,connect,stopped,process_callback):
    if stopped.is_set():raise InputCheckError('检查已取消')
    helper=(root/'tools/reconstruction_core.py').read_text(encoding='utf-8')
    code='import sys,types\nm=types.ModuleType("reconstruction_core");sys.modules["reconstruction_core"]=m\nexec('+repr(helper)+',m.__dict__)\n'
    code+=(root/'tools/reconstruction_pose.py').read_text(encoding='utf-8')+'\n'
    code+='CONNECT_CAMERA='+repr(bool(connect))+'\n'+(root/'tools/reconstruction_input_probe.py').read_text(encoding='utf-8')
    command=['ssh','-F',str(root/'config/readonly_ssh_config'),'cyberdog-current-nx',
             'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                             text=True,encoding='utf-8',cwd=str(root))
    process_callback(process)
    try:
        output,stderr=process.communicate(code,timeout=45)
    except subprocess.TimeoutExpired:
        process.kill();process.communicate()
        raise InputCheckError('相机检查超时；没有自动重试')
    finally:process_callback(None)
    if stopped.is_set():raise InputCheckError('检查已取消；若初始化已发出，请重新检查相机状态')
    if process.returncode:raise InputCheckError('机器人输入检查连接失败',stderr[-2000:])
    try:
        result=json.loads(output.strip().splitlines()[-1])
        if not isinstance(result,dict):raise ValueError()
    except (ValueError,IndexError):raise InputCheckError('相机检查没有返回有效结果')
    return result


def prepare_inputs(root,connect,stopped,process_callback):
    result=probe(root,connect,stopped,process_callback)
    if (connect and not result.get('ready') and result.get('camera_ready')
            and result.get('task')=={'status':101,'code':0} and not stopped.is_set()):
        # Only the explicit Connect button may start stock scanning. The normal
        # capture-start check is read-only and never takes over another task.
        from mapping_bridge import MappingBridge
        name='computer_room_'+time.strftime('%Y%m%d_%H%M%S')
        bridge=MappingBridge(root);bridge.name=name
        bridge.run({'action':'start','map_name':name,'stop_localization':False,
                    'mapping_sensor':'lidar','navigation_sensor':'lidar'})
        mapping=bridge.report()
        if stopped.is_set():raise InputCheckError('准备已取消；扫描请求可能已生效，请重新检查')
        if not any(e.get('event')=='SCANNING' for e in mapping['events']):
            error=next((e.get('message') for e in mapping['events'] if e.get('event')=='ERROR'),'扫描输入未就绪')
            result.update(ready=False,message=error,mapping=mapping)
        else:
            initial=result
            result=probe(root,False,stopped,process_callback)
            result['mapping']=mapping
            result['initial_camera_check']=initial
    folder=root/'evidence/camera_checks';folder.mkdir(parents=True,exist_ok=True)
    result.update(checked_unix=time.time(),operation='connect' if connect else 'check')
    (folder/('%d.json'%time.time_ns())).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result
