"""One reviewed, backed-up stock map selection repair; never sends motion."""
import json,shlex,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
scope=json.loads((ROOT/'config/scope.json').read_text(encoding='utf-8'))
if '--apply-reviewed-map-selection' not in sys.argv or scope.get('motion_enabled') or scope.get('manual_motion_enabled'):
    raise RuntimeError('Explicit repair argument and BOTH motion permissions disabled required')
code='MAP_NAME='+repr(scope['computer_map_name'])+'\n'+(ROOT/'tools/map_inventory.py').read_text(encoding='utf-8')+r'''
import json,rclpy
from protocol.srv import AlgoTaskStatus,StopAlgoTask
NS='/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node=rclpy.create_node('computer_active_map_repair',enable_rosout=False,start_parameter_services=False)
def call(name,typ,request):
    client=node.create_client(typ,NS+'/'+name)
    try:
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Service unavailable')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=40)
        if not future.done() or future.result() is None:raise RuntimeError('Uncertain repair response; no retry')
        return future.result()
    finally:node.destroy_client(client)
try:
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    print(json.dumps({'event':'STATUS','status':status.status,'code':status.code}),flush=True)
    if status.code!=0 or status.status not in (8,101,6):raise RuntimeError('Another task owns robot; no map file changed')
    if status.status==8:
        request=StopAlgoTask.Request();request.task_id=0
        reply=call('stop_algo_task',StopAlgoTask,request)
        print(json.dumps({'event':'LOCALIZATION_ENDED','result':reply.result,'code':reply.code}),flush=True)
        if reply.result!=0 or reply.code!=0:raise RuntimeError('Localization not stopped; no map file changed')
    status=call('algo_task_status',AlgoTaskStatus,AlgoTaskStatus.Request())
    if status.code!=0 or status.status not in (101,6):raise RuntimeError('Task not idle; no file changed')
    print(json.dumps(dict(event='MAP_FILES_SELECTED',**archive_other_maps('/home/mi/mapping',MAP_NAME))),flush=True)
finally:node.destroy_node();rclpy.shutdown()
'''
command=['ssh','-F',str(ROOT/'config/readonly_ssh_config'),'cyberdog-current-nx',
         'source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
result=subprocess.run(command,input=code,text=True,encoding='utf-8',capture_output=True,timeout=80)
(ROOT/'evidence/active_map_repair_20261009.jsonl').write_text(result.stdout,encoding='utf-8')
(ROOT/'evidence/active_map_repair_stderr_20261009.txt').write_text(result.stderr,encoding='utf-8')
print(result.stdout);print(result.stderr,file=sys.stderr)
sys.exit(result.returncode)
