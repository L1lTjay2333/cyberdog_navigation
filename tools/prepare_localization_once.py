"""Stock indoor mapping only. No pose goals, velocity, or control parameters."""
import json
import sys
import time
import rclpy
from rclpy.action import ActionClient
from protocol.action import Navigation
from protocol.srv import AlgoTaskStatus

NS = '/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node = rclpy.create_node('flat_navigation_localization_prepare', enable_rosout=False, start_parameter_services=False)
client = None

def log(**item):
    item['unix'] = time.time()
    print(json.dumps(item), flush=True)

try:
    status = node.create_client(AlgoTaskStatus, NS+'/algo_task_status')
    if not status.wait_for_service(timeout_sec=5):
        raise RuntimeError('Status service unavailable')
    f = status.call_async(AlgoTaskStatus.Request())
    rclpy.spin_until_future_complete(node, f, timeout_sec=8)
    if not f.done() or f.result() is None:
        raise RuntimeError('Status read timeout')
    log(status=int(f.result().status), code=int(f.result().code))
    if f.result().status not in (AlgoTaskStatus.Response.IDLE, 6) or f.result().code != 0:
        raise RuntimeError('Task manager is not idle; no start sent')
    client = ActionClient(node, Navigation, NS+'/start_algo_task')
    if not client.wait_for_server(timeout_sec=5):
        raise RuntimeError('Mapping action unavailable')
    goal = Navigation.Goal()
    goal.nav_type = Navigation.Goal.NAVIGATION_TYPE_START_LOCALIZATION
    goal.outdoor = False
    goal.poses = []
    goal.map_name = 'room_navigation_20261007'
    log(event='SEND_ONCE', nav_type=goal.nav_type, outdoor=False, poses=[])
    f = client.send_goal_async(goal, feedback_callback=lambda x: log(feedback_code=x.feedback.feedback_code, feedback_msg=x.feedback.feedback_msg))
    rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if not f.done() or f.result() is None:
        raise RuntimeError('Uncertain start acknowledgement; do not retry')
    handle = f.result()
    log(event='ACK', accepted=handle.accepted)
    if not handle.accepted:
        raise RuntimeError('Start rejected; do not retry')
    result = handle.get_result_async()
    started = time.monotonic()
    while time.monotonic()-started < 130 and not result.done():
        rclpy.spin_once(node, timeout_sec=0.2)
    if result.done():
        log(event='RESULT', status=result.result().status, result=result.result().result.result)
    else:
        log(event='MONITOR_END', note='Localization task may remain active; client did not cancel or send another goal')
except Exception as exc:
    log(event='ERROR', message=str(exc))
    sys.exit(2)
finally:
    if client is not None:
        client.destroy()
    node.destroy_node()
    rclpy.shutdown()
