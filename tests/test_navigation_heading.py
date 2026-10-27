import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from navigation_heading import initial_heading,heading_error,turn_velocity


class HeadingTests(unittest.TestCase):
    def test_behind_has_nonzero_turn(self):
        target=initial_heading([{'x':-.7,'y':0}],0,0,0)
        self.assertEqual(abs(turn_velocity(heading_error(target,0))),.5)

    def test_path_tangent_not_goal_through_obstacle(self):
        self.assertAlmostEqual(initial_heading([{'x':0,'y':0},{'x':0,'y':.6},{'x':2,'y':0}],0,0,0),math.pi/2)

    def test_short_path_does_not_invent_heading(self):
        self.assertEqual(initial_heading([{'x':.1,'y':.1}],0,0,1),1)

    def test_wrap_and_turn_deadband(self):
        self.assertAlmostEqual(heading_error(-math.pi+.05,math.pi-.05),.1)
        self.assertEqual(turn_velocity(.1),0)
        self.assertEqual(turn_velocity(-.3),-.28)

    def fixture(self,scan_age=0,progress=True,motion_status=None):
        source=Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='align_initial_heading')
        clock=[100.];yaw=[0.];sent=[];events=[];destroy=[]
        class Servo:
            APP=1;SERVO_START=1;SERVO_DATA=2;SERVO_END=3
        class Node:
            def create_publisher(self,*a):return S(publish=lambda m:sent.append(m))
            def create_subscription(self,*a):return object()
            def destroy_subscription(self,s):destroy.append('subscription')
            def destroy_publisher(self,p):destroy.append('publisher')
        def transform(*a):
            return S(header=S(stamp=S(sec=int(clock[0]),nanosec=int((clock[0]%1)*1e9))),
                transform=S(translation=S(x=0,y=0),rotation=S(w=math.cos(yaw[0]/2),z=math.sin(yaw[0]/2),x=0,y=0)))
        def spin(*a,**kw):
            clock[0]+=.05
            observations['motion_received']=clock[0]
            if scan_age==0:observations['scan']=clock[0]
            if progress and sent and sent[-1].cmd_type==Servo.SERVO_DATA:yaw[0]+=sent[-1].vel_des[2]*.05
        observations={'planned_points':[{'x':-.7,'y':0}], 'scan':100-scan_age,
            'scan_message':S(ranges=[2.]*20,range_min=.2,range_max=5.5),
            'motion_received':100.,'motion_status':motion_status or S(motion_id=112,switch_status=0,
                ori_error=0,footpos_error=0,motor_error=[0]*12)}
        env={'math':math,'time':S(time=lambda:clock[0],monotonic=lambda:clock[0]),
            'buffer':S(lookup_transform=transform),'rclpy':S(time=S(Time=lambda:None),spin_once=spin),
            'node':Node(),'NS':'/test','observations':observations,'initial_heading':initial_heading,
            'heading_error':heading_error,'turn_velocity':turn_velocity,'operator_ready':lambda:None,
            'battery_ready':lambda:None,'MotionServoCmd':Servo,'MotionStatus':object,
            'AlgoTaskStatus':S(Request=lambda:None),'call':lambda *a:S(status=8,code=0),
            'emit':lambda **row:events.append(row)}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'align','exec'),env)
        return env,sent,events,destroy

    def test_turn_start_data_end_then_complete(self):
        env,sent,events,destroy=self.fixture();env['align_initial_heading']()
        self.assertEqual(sent[0].cmd_type,1);self.assertEqual(sent[-1].cmd_type,3)
        self.assertTrue(all(m.vel_des[:2]==[0.,0.] for m in sent))
        self.assertTrue(any(e['event']=='PRETURN_COMPLETE' for e in events))
        self.assertEqual(destroy,['subscription','publisher'])

    def test_unused_motor_sentinel_does_not_block_turn(self):
        status=S(motion_id=112,switch_status=1,ori_error=0,footpos_error=0,
                 motor_error=[-2147483648]*12)
        env,sent,events,_=self.fixture(motion_status=status)
        env['align_initial_heading']()
        self.assertTrue(any(e['event']=='PRETURN_COMPLETE' for e in events))
        self.assertEqual(sent[-1].cmd_type,3)

    def test_real_motor_error_still_blocks_before_motion(self):
        status=S(motion_id=112,switch_status=0,ori_error=0,footpos_error=0,
                 motor_error=[-2147483648]*11+[7])
        env,sent,events,_=self.fixture(motion_status=status)
        with self.assertRaisesRegex(RuntimeError,'posture/motor not ready'):
            env['align_initial_heading']()
        self.assertFalse(sent)
        self.assertEqual(events[-1]['motor_faults'],[7])

    def test_stale_scan_sends_no_motion(self):
        env,sent,_,_=self.fixture(scan_age=3)
        with self.assertRaisesRegex(RuntimeError,'Scan expired'):env['align_initial_heading']()
        self.assertFalse(sent)

    def test_close_return_defers_turn_to_native_navigation(self):
        env,sent,events,destroy=self.fixture()
        env['observations']['scan_message'].ranges=[.2]+[2.]*19
        self.assertFalse(env['align_initial_heading']())
        self.assertFalse(sent)
        self.assertEqual(events[-1]['event'],'PRETURN_DEFERRED_TO_NAV2')
        self.assertEqual(destroy,['subscription','publisher'])

    def test_no_turn_progress_stops_instead_of_marching_forever(self):
        env,sent,_,_=self.fixture(progress=False)
        with self.assertRaisesRegex(RuntimeError,'no measured angular progress'):env['align_initial_heading']()
        self.assertEqual(sent[-1].cmd_type,3)
