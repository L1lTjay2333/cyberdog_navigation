"""Exercise the actual robot-side relocalization function without ROS/motion."""
import ast
from pathlib import Path
import time
import types
import unittest

def item(**kwargs):return types.SimpleNamespace(**kwargs)

class RelocalizationGateTests(unittest.TestCase):
    def run_gate(self,status=8,reset_ok=True,localization_ok=True,cancel=False,final_statuses=None,camera_ok=True,delayed_feedback=False,depth=False):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text(encoding='utf-8'))
        body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('operator_ready','refresh_localization')]
        operations=[];events=[];statuses=iter(([status,101,101] if status==8 else [status,status,status])+(final_statuses or [8]));pending_feedback=[]
        class Future:
            def __init__(self,value):self.value=value
            def done(self):return True
            def result(self):return self.value
        class Client:
            def wait_for_server(self,**_):return True
            def send_goal_async(self,goal,feedback_callback):
                operations.append(('goal',goal.nav_type))
                self_test.last_map_name=goal.map_name
                feedback=item(feedback=item(feedback_code=3108 if localization_ok else 3106))
                if delayed_feedback:pending_feedback.append((feedback_callback,feedback))
                else:feedback_callback(feedback)
                result=item(status=4 if localization_ok else 6,result=item(result=0 if localization_ok else 5))
                def cancel_goal():
                    operations.append(('cancel_localization',None))
                    return Future(item(goals_canceling=[item()]))
                handle=item(accepted=True,get_result_async=lambda:Future(result),cancel_goal_async=cancel_goal)
                return Future(handle)
            def destroy(self):pass
        def call(service,kind,request):
            operations.append((service,getattr(request,'task_id',None)))
            if service=='algo_task_status':return item(status=next(statuses),code=0)
            return item(result=0 if reset_ok else 1,code=0)
        self_test=self
        def prepare_camera(*_):
            if not camera_ok:raise RuntimeError('Camera service unresponsive')
        env={'assert_active_map':lambda *_:None,'prepare_stock_camera':prepare_camera,'ensure_depth_slam':lambda *_args,**_kw:None,
             'REQUEST':{'map_name':'computer_room_test','navigation_sensor':'depth_camera' if depth else 'lidar'},'emit':lambda **event:events.append(event),'heartbeat':{'stop':cancel,'last':time.monotonic()},'time':time,
             'call':call,'AlgoTaskStatus':item(Request=lambda:item()),'StopAlgoTask':item(Request=lambda:item()),
             'Navigation':item(Goal=lambda:item()),'ActionClient':lambda *_:Client(),'NS':'/test','node':None,
             'rclpy':item(spin_until_future_complete=lambda *_args,**_kw:None,
                          spin_once=lambda *_args,**_kw:deliver_feedback())}
        def deliver_feedback():
            if pending_feedback:
                callback,feedback=pending_feedback.pop(0);callback(feedback)
        exec(compile(ast.Module(body=body,type_ignores=[]),'relocalization_gate','exec'),env)
        try:begun=env['refresh_localization']();error=None
        except RuntimeError as exc:begun=None;error=str(exc)
        return operations,events,begun,error
    def test_every_attempt_resets_and_relocalizes_once(self):
        for _ in range(2):
            operations,events,begun,error=self.run_gate()
            self.assertIsNone(error);self.assertIsNotNone(begun)
            self.assertEqual(operations,[('algo_task_status',None),('stop_algo_task',0),('algo_task_status',None),('algo_task_status',None),('goal',7),('algo_task_status',None)])
            self.assertEqual(events[-1]['event'],'RELOCALIZATION_SUCCESS')
    def test_busy_robot_is_never_reset(self):
        operations,_,begun,error=self.run_gate(status=1)
        self.assertIsNotNone(error);self.assertIsNone(begun)
        self.assertEqual(operations,[('algo_task_status',None)])
    def test_success_result_waits_for_manager_state_without_resending(self):
        operations,events,begun,error=self.run_gate(final_statuses=[7,8])
        self.assertIsNone(error)
        self.assertIsNotNone(begun)
        self.assertEqual([op for op in operations if op[0]=='goal'],[('goal',7)])
        self.assertFalse(any(op[0]=='cancel_localization' for op in operations))
        self.assertEqual(events[-1]['event'],'RELOCALIZATION_SUCCESS')
    def test_success_feedback_arriving_after_result_is_not_false_failure(self):
        operations,events,begun,error=self.run_gate(delayed_feedback=True)
        self.assertIsNone(error)
        self.assertIsNotNone(begun)
        self.assertEqual([op for op in operations if op[0]=='goal'],[('goal',7)])
        self.assertFalse(any(op[0]=='cancel_localization' for op in operations))
    def test_failed_localization_state_starts_new_attempt_without_rejected_reset(self):
        operations,events,begun,error=self.run_gate(status=6,depth=True)
        self.assertIsNone(error)
        self.assertIsNotNone(begun)
        self.assertFalse(any(op[0]=='stop_algo_task' for op in operations))
        self.assertEqual([op for op in operations if op[0]=='goal'],[('goal',7)])
    def test_uses_current_computer_map_name(self):
        _,_,_,error=self.run_gate()
        self.assertIsNone(error)
        self.assertEqual(self.last_map_name,'computer_room_test')
    def test_reset_failure_does_not_send_localization_or_navigation(self):
        operations,_,begun,error=self.run_gate(reset_ok=False)
        self.assertIsNotNone(error);self.assertIsNone(begun)
        self.assertFalse(any(op[0]=='goal' for op in operations))
    def test_localization_failure_never_reports_success(self):
        operations,events,begun,error=self.run_gate(localization_ok=False)
        self.assertIsNotNone(error);self.assertIsNone(begun)
        self.assertNotIn('RELOCALIZATION_SUCCESS',[e['event'] for e in events])
        self.assertEqual([op for op in operations if op[0]=='goal'],[('goal',7)])
        self.assertEqual([op for op in operations if op[0]=='cancel_localization'],[('cancel_localization',None)])
        self.assertIn({'event':'LOCALIZATION_CANCEL','confirmed':True},events)
    def test_cancel_before_preparation_sends_nothing(self):
        operations,_,_,error=self.run_gate(cancel=True)
        self.assertIsNotNone(error);self.assertEqual(operations,[])

    def test_camera_failure_never_submits_algorithm(self):
        operations,_,begun,error=self.run_gate(status=101,camera_ok=False)
        self.assertIsNotNone(error);self.assertIsNone(begun)
        self.assertFalse(any(op[0]=='goal' for op in operations))

if __name__=='__main__':unittest.main()
