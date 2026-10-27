import ast,math,time,unittest
from pathlib import Path
from unittest.mock import patch
from exploration_policy import checked_velocity,sensor_problem,trajectory_clear,track,overlay_depth
from exploration_bridge import ExplorationBridge
from test_unknown_goal import grid

def health():
    return {'sensor_mode':'supervised_four_way_trial','auxiliary_task':5,'task':5,'task_time':10.,
        'status_time':10.,'status':{'motion_id':112,'switch_status':0,'ori_error':0,'footpos_error':0,'motor_error':[0]*12},
        'battery_time':10.,'battery':{'percent':44,'wired_charging':False,'wireless_charging':False},
        'scan_time':10.,'depth_time':10.,'depth_valid':True,'depth_min':3.}

class ExplorationPolicyTests(unittest.TestCase):
    def test_manual_camera_callbacks_only_keep_latest_frame(self):
        import threading
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'tools/manual_robot_session.py').read_text(encoding='utf-8'))
        callbacks=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('queue_depth_image','queue_native_cloud')]
        env={'depth_pending':{},'depth_pending_lock':threading.Lock()}
        exec(compile(ast.Module(body=callbacks,type_ignores=[]),'sensor_queue_callbacks','exec'),env)
        for i in range(100):env['queue_depth_image'](i);env['queue_native_cloud'](i)
        self.assertEqual(env['depth_pending'],{'image':99,'cloud':99})
        for callback in callbacks:
            self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in ('depth','native_cloud') for n in ast.walk(callback)))
    def test_actual_probe_main_never_publishes_velocity(self):
        import types,threading
        from unittest.mock import Mock
        from pose_history import PoseHistory
        from cloud_geometry import transform_xyz,FootprintObservation
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'tools/exploration_robot_session.py').read_text(encoding='utf-8'))
        main=next(n for n in tree.body if isinstance(n,ast.Try))
        clock=types.SimpleNamespace(value=100.)
        def spin(*_,**__):clock.value+=.02
        def mono():return clock.value
        g=grid(False);g['received_unix']=100.
        h=health();h.update(status_time=100.,task_time=100.,battery_time=100.,scan_time=100.,depth_time=100.,depth_stamp_ns=100000000000,depth_points_base=[])
        history=PoseHistory();history.add(100.,[1.,1.5,0],[0,0,0,1],'laser_odom')
        send=Mock();events=[];ticks=iter([True,True,True,False])
        env={'time':types.SimpleNamespace(monotonic=mono,time=lambda:clock.value,sleep=lambda _:spin()),'math':math,'live':[True],
            'rclpy':types.SimpleNamespace(ok=lambda:next(ticks),spin_once=spin,shutdown=Mock()),'node':types.SimpleNamespace(get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=100000000000)),destroy_node=Mock()),
            'target':None,'execute':False,'REQUEST':{},'maps':{'grid':g},'pose_history':history,'health':h,
            'compensated_pose':types.SimpleNamespace(latest=history.latest,at=lambda stamp,now:history.at(stamp)),
            'pose_diagnostic':{},
            'scan_turn':False,'footprint_observation':FootprintObservation(),'running':[True],'sensor_thread':Mock(),
            'dynamic_key':None,'dynamic_cache':None,'scan_pause_at':None,'scan_odometry':None,
            'begin':100.,'heartbeat':[100.],'cancelled':[False],'MAX_RUNTIME':120.,'MAX_TRAVEL':5.,
            'sensor_problem':sensor_problem,'transform_xyz':transform_xyz,'overlay_depth':overlay_depth,
            'owner_conflict':[False],'stop':Mock(),'send':send,'sent':[],'emit':lambda event,**data:events.append({'event':event,**data}),
            'ready_emitted':False,'last_feedback':-100.,'number':lambda x:x,'active':False}
        exec(compile(ast.Module(body=[main],type_ignores=[]),'actual_probe_loop','exec'),env)
        send.assert_not_called();self.assertTrue(any(e['event']=='READY' for e in events));self.assertEqual(events[-1]['event'],'EXITED_NO_MOTION')
    def test_velocity_rejects_reverse_lateral_high_speed_and_nan(self):
        for v in ([-.1,0,0],[.2,0,0],[0,.1,0],[0,0,.26],[True,0,0],[float('nan'),0,0]):
            with self.assertRaises(ValueError):checked_velocity(v)
        self.assertEqual(checked_velocity([.15,0,.25]),(.15,0,.25))
    def test_forward_track(self):
        g=grid(False);pose={'x':1.05,'y':1.55,'yaw':0,'frame':'laser_odom'}
        route={'frame':'laser_odom','points':[{'x':1.05,'y':1.55},{'x':1.25,'y':1.55}]}
        v,reason=track(g,pose,route,health(),10.1)
        self.assertEqual(reason,'');self.assertGreater(v[0],0);self.assertLessEqual(v[0],.15)
    def test_owned_in_place_scan_does_not_require_forward_roi_but_keeps_other_guards(self):
        h=health();h.update(depth_valid=False,depth_time=0)
        self.assertEqual(sensor_problem(h,10.1,require_depth=False),'')
        self.assertTrue(sensor_problem(h,10.1))
        h['scan_time']=0;self.assertTrue(sensor_problem(h,10.1,require_depth=False))
    def test_turn_before_forward_not_corner_cut(self):
        g=grid(False);pose={'x':1.05,'y':1.55,'yaw':0,'frame':'laser_odom'}
        route={'frame':'laser_odom','points':[{'x':1.05,'y':1.8}]}
        v,reason=track(g,pose,route,health(),10.1)
        self.assertEqual(reason,'');self.assertEqual(v[0],0);self.assertEqual(v[2],.25)
    def test_old_route_does_not_make_robot_turn_back_to_start(self):
        g=grid(False);pose={'x':1.30,'y':1.55,'yaw':0,'frame':'laser_odom'}
        route={'frame':'laser_odom','points':[{'x':1.+i*.1,'y':1.55} for i in range(10)]}
        v,reason=track(g,pose,route,health(),10.1)
        self.assertEqual(reason,'');self.assertGreater(v[0],0);self.assertEqual(v[2],0)
    def test_predictive_path_blocks_unknown_and_obstacle(self):
        g=grid(False);pose={'x':1.05,'y':1.55,'yaw':0,'frame':'laser_odom'}
        self.assertTrue(trajectory_clear(g,pose,[.15,0,0]))
        for value in (-1,100):
            g['data'][15*40+15]=value
            self.assertFalse(trajectory_clear(g,pose,[.15,0,0]))
    def test_all_sensor_and_charge_failures_block(self):
        for change in ('charge','invalid_battery','battery_age','depth_age','scan_age','task','motor'):
            h=health()
            if change=='charge':h['battery']['wired_charging']=True
            if change=='invalid_battery':h['battery']['percent']=-1
            if change=='battery_age':h['battery_time']=0
            if change=='depth_age':h['depth_time']=0
            if change=='scan_age':h['scan_time']=0
            if change=='task':h['task']=101
            if change=='motor':h['status']['motor_error']=[5]
            self.assertTrue(sensor_problem(h,10.1),change)
    def test_low_percentage_does_not_block_exploration(self):
        for percent in (0,9,19):
            h=health();h['battery']['percent']=percent
            self.assertEqual(sensor_problem(h,10.1),'')
    def test_depth_overlay_does_not_clear_lidar_or_mutate_input(self):
        g=grid(False);g['data'][5]=100
        result=overlay_depth(g,None,[[1.05,1.55,.4]])
        self.assertEqual(result['data'][15*40+10],100)
        self.assertEqual(result['data'][5],100);self.assertEqual(g['data'][15*40+10],0)
    def test_close_depth_limits_forward_not_disable_still_turn(self):
        g=grid(False);pose={'x':1.05,'y':1.55,'yaw':0,'frame':'laser_odom'};h=health();h['depth_min']=.46
        v,reason=track(g,pose,{'frame':'laser_odom','points':[{'x':1.3,'y':1.55}]},h,10.1)
        self.assertEqual(reason,'');self.assertLessEqual(v[0],.011)
    def test_code_bootstrap_does_not_include_manual_loop(self):
        root=Path(__file__).resolve().parents[1]
        source=(root/'tools/manual_robot_session.py').read_text(encoding='utf-8')
        prefix=source.split('\nensure_idle_camera()\n')[0]
        ast.parse(prefix)
        self.assertNotIn('threading.Thread(target=reader',prefix)
        self.assertNotIn('while running[0]',prefix)
        ast.parse((root/'tools/exploration_robot_session.py').read_text(encoding='utf-8'),feature_version=(3,6))
    def test_bridge_no_auto_start_or_takeover(self):
        root=Path(__file__).resolve().parents[1];bridge=ExplorationBridge(root)
        self.assertFalse(bridge.report()['active'])
        with patch.object(bridge,'deployment',return_value={'remote_dir':'/home/mi/cyberdog_exploration_20261009_000000','robot_python_compile':True}),patch('exploration_bridge.threading.Thread') as thread:
            bridge.start({'target':{'x':1}},'test_owner_123',False)
            self.assertFalse(bridge.executing);self.assertTrue(bridge.active)
            with self.assertRaises(ValueError):bridge.heartbeat('another_owner')
            bridge.heartbeat('test_owner_123');bridge.stop();self.assertTrue(bridge.cancel_event.is_set())
            with self.assertRaises(ValueError):bridge.start({},'another_owner',True)
            self.assertEqual(thread.call_count,1)

if __name__=='__main__':unittest.main()
