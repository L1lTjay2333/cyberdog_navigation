import unittest
from navigation_decision import NavigationDecision,validate_preparation,validate_goal_distance,motion_problem

class DecisionTests(unittest.TestCase):
    def test_failed_preparation_can_retry_after_inputs_recover(self):
        self.t.update(active=False,goal=self.goal,session_started_unix=100,
                      events=[{'event':'ERROR','message':'navigation map missing'}])
        self.s['pose']['received_unix']=90
        self.assertFalse(self.evaluate()['can_start'])
        self.refresh()
        result=self.evaluate();self.assertTrue(result['can_start'])
        self.assertIn('navigation map missing',result['message'])

    def test_submitted_goal_failure_cannot_be_auto_reclassified_ready(self):
        self.t.update(active=False,goal=self.goal,session_started_unix=100,
                      events=[{'event':'SEND_ONCE'},{'event':'ERROR','message':'transport failure'}])
        self.assertFalse(self.evaluate()['can_start'])

    def test_map_heading_jump_without_odometry_turn_stops(self):
        self.activate()
        self.s['odom']={'x':0.,'y':0.,'yaw':0.,'frame':'odom','received_unix':100}
        self.evaluate()
        self.now+=.2;self.refresh();self.s['odom']['received_unix']=self.now
        self.s['pose']['yaw']=1.
        result=self.evaluate()
        self.assertTrue(result['stop_required']);self.assertIn('里程计',result['message'])

    def test_real_rotation_and_angle_wrap_do_not_look_like_jump(self):
        self.activate();self.s['pose']['yaw']=3.1
        self.s['odom']={'x':0.,'y':0.,'yaw':3.1,'frame':'odom','received_unix':100}
        self.evaluate()
        self.now+=.2;self.refresh();self.s['odom']['received_unix']=self.now
        self.s['pose']['yaw']=-3.1;self.s['odom']['yaw']=-3.1
        self.assertFalse(self.evaluate()['stop_required'])

    def test_unsynchronized_odometry_is_not_used_for_jump_detection(self):
        self.activate()
        self.s['odom']={'x':0.,'y':0.,'yaw':0.,'frame':'odom','received_unix':98}
        self.evaluate();self.now+=.2;self.refresh();self.s['pose']['yaw']=1.
        self.assertFalse(self.evaluate()['stop_required'])

    def test_position_correction_and_rotated_odometry_frame(self):
        self.activate()
        self.s['pose']['yaw']=1.5707963267948966
        self.s['odom']={'x':10.,'y':20.,'yaw':0.,'frame':'odom','received_unix':100}
        self.evaluate()
        self.now+=.2;self.refresh();self.s['odom']['received_unix']=self.now
        self.s['odom']['x']+=.1;self.s['pose']['y']+=.1
        self.assertFalse(self.evaluate()['stop_required'])
        self.now+=.2;self.refresh();self.s['odom']['received_unix']=self.now
        self.s['pose']['x']+=.4
        self.assertTrue(self.evaluate()['stop_required'])

    def test_whole_map_mode_has_no_short_distance_cap(self):
        validate_goal_distance(self.s['pose'],{'x':50,'y':50},None)
    def test_accepted_navigation_transition_not_false_disconnect(self):
        self.t.update(active=True,goal=self.goal,session_started_unix=100,
            events=[{'event':'SEND_ONCE','unix':100},{'event':'ACK','accepted':True}])
        self.s['task']['status']=101;self.s['pose']=None
        self.assertEqual(self.evaluate()['stage'],'STARTING')
        self.assertFalse(self.evaluate()['stop_required'])
        self.s['connection']='disconnected'
        self.assertTrue(self.evaluate()['stop_required'])
        self.s['connection']='connected';self.now=111;self.refresh_without_pose()
        self.assertTrue(self.evaluate()['stop_required'])
    def refresh_without_pose(self):
        self.s['local_received_unix']=self.now;self.s['emitted_unix']=self.now
        for key in ('task','scan','battery'):self.s[key]['received_unix']=self.now
    def test_idle_before_first_remote_event_does_not_cancel_preparation(self):
        self.t.update(active=True,goal=None,preparing_localization=True)
        self.s['task']['status']=101;self.s['pose']=None;self.s['scan']=None;self.selection=None
        result=self.evaluate()
        self.assertEqual(result['stage'],'RELOCALIZING');self.assertFalse(result['stop_required'])
    def setUp(self):
        self.now=100
        self.s={'connection':'connected','local_received_unix':100,'emitted_unix':100,
            'task':{'status':8,'received_unix':100},
            'pose':{'x':.5,'y':.5,'yaw':0,'frame':'map','received_unix':100},
            'scan':{'received_unix':100,'finite_ranges':100},
            'battery':{'percent':50,'received_unix':100,'wired_charging':False,'wireless_charging':False},
            'map':{'frame':'map','width':5,'height':5,'resolution':.5,'origin':{'x':0,'y':0,'qz':0,'qw':1},'data':[0]*25}}
        self.goal={'x':1.4,'y':.5,'frame':'map'}
        self.route={'kind':'stock_nav2_preview','frame':'map','points':[{'x':.5,'y':.5},{'x':1.4,'y':.5}]}
        self.selection=validate_preparation(self.s,self.goal,self.route,100)
        self.t={'enabled':True,'active':False,'goal':None,'events':[]}
        self.d=NavigationDecision()
    def evaluate(self):return self.d.evaluate(self.s,self.t,self.selection,self.now)
    def activate(self):
        self.t.update(active=True,goal=self.goal,session_started_unix=100,events=[{'event':'FEEDBACK','code':307}])
        self.s['task']['status']=1
    def refresh(self):
        self.s['local_received_unix']=self.now;self.s['emitted_unix']=self.now
        for key in ('task','pose','scan','battery'):self.s[key]['received_unix']=self.now

    def test_preparation_reuses_localization_and_does_not_claim_relocalizing(self):
        self.t.update(active=True,goal=self.goal,preparing_navigation=True,session_started_unix=100)
        self.assertEqual(self.evaluate()['stage'],'REPLANNING')
        self.assertFalse(self.evaluate()['stop_required'])

    def test_real_localization_retry_is_not_a_web_disconnect(self):
        self.t.update(active=True,goal=None,preparing_localization=True,session_started_unix=100,
            events=[{'event':'RELOCALIZATION_FEEDBACK','code':3107}])
        self.selection=None;self.s['task']['status']=7;self.s['pose']=None
        result=self.evaluate();self.assertIn('3107',result['message']);self.assertFalse(result['stop_required'])

    def test_fault_exposes_actual_robot_message(self):
        self.t.update(active=False,goal=self.goal,session_started_unix=100,events=[{'event':'ERROR','message':'Goal exceeds test limit'}])
        self.assertIn('Goal exceeds test limit',self.evaluate()['message'])

    def test_distance_checked_before_transport(self):
        validate_goal_distance(self.s['pose'],self.goal,2.8)
        with self.assertRaisesRegex(ValueError,'2.8'):
            validate_goal_distance(self.s['pose'],{'x':4,'y':.5},2.8)

    def test_charging_blocks_but_low_percentage_does_not(self):
        self.s['battery']['wired_charging']=True
        self.assertIn('充电',motion_problem(self.s,self.now))
        self.assertFalse(self.evaluate()['can_start'])
        validate_preparation(self.s,self.goal,self.route,self.now)
        self.s['battery']['wired_charging']=False;self.s['battery']['percent']=9
        self.assertIsNone(motion_problem(self.s,self.now))
        self.assertTrue(self.evaluate()['can_start'])
        self.s['battery']['percent']=0
        self.assertTrue(self.evaluate()['can_start'])

    def test_invalid_or_missing_battery_feedback_still_blocks(self):
        self.s['battery']['percent']=float('nan')
        self.assertIn('无效',motion_problem(self.s,self.now))
        self.s['battery']['percent']=9;self.s['battery']['received_unix']=90
        self.assertIn('未就绪',motion_problem(self.s,self.now))

    def test_delayed_emission_cannot_fake_live_localization(self):
        self.s['emitted_unix']=90
        self.assertFalse(self.evaluate()['can_start'])
    def test_ready_requires_motion_unlock(self):
        self.assertTrue(self.evaluate()['can_start'])
        self.t['enabled']=False;self.assertFalse(self.evaluate()['can_start'])
    def test_robot_outside_map_cannot_start_or_keep_moving(self):
        self.s['pose']['x']=-.3
        self.assertFalse(self.evaluate()['can_start'])
        self.activate()
        self.assertTrue(self.evaluate()['stop_required'])
    def test_unknown_goal_and_wrong_planner_endpoint_rejected(self):
        self.s['map']['data'][7]=-1
        with self.assertRaises(ValueError):validate_preparation(self.s,self.goal,self.route,100)
        self.s['map']['data'][7]=0;self.route['points'][-1]['x']=.6
        with self.assertRaises(ValueError):validate_preparation(self.s,self.goal,self.route,100)
    def test_near_goal_does_not_claim_arrival_before_result(self):
        self.activate();self.s['pose']['x']=1.35
        self.assertEqual(self.evaluate()['stage'],'EXECUTING')
        self.t['events'].append({'event':'RESULT','status':4,'result':0})
        self.s['task']['status']=8
        self.assertEqual(self.evaluate()['stage'],'ARRIVED')
    def test_success_with_bad_position_needs_verification(self):
        self.activate();self.t['events'].append({'event':'RESULT','status':4,'result':0});self.s['task']['status']=8
        self.assertEqual(self.evaluate()['stage'],'VERIFY')
        self.s['pose']['x']=1.4;self.s['pose']['received_unix']=90
        self.assertEqual(self.evaluate()['stage'],'VERIFY')
    def test_cancel_is_not_arrival(self):
        self.activate();self.t['events'] += [{'event':'STOP_RESULT','result':0,'code':0},{'event':'RESULT','status':6,'result':5}]
        self.assertEqual(self.evaluate()['stage'],'CANCELLED')
        self.t['events'].append({'event':'STOP_TASK_TERMINAL','status':5})
        self.assertEqual(self.evaluate()['stage'],'CANCELLED')
    def test_stop_ack_without_terminal_status_never_confirms_parking(self):
        self.activate();self.t['events'].append({'event':'STOP_SERVICE_ACK','success':True})
        self.assertEqual(self.evaluate()['stage'],'STOPPING')
        self.t['active']=False
        self.assertEqual(self.evaluate()['stage'],'STOP_UNCONFIRMED')
    def test_verified_idle_without_terminal_callback_can_finish_stop(self):
        self.activate();self.t['events'] += [{'event':'STOP_SERVICE_ACK','success':True},{'event':'STOP_VERIFIED_IDLE'}]
        self.assertEqual(self.evaluate()['stage'],'CANCELLED')
        self.assertIn('已核实停车',self.evaluate()['message'])
    def test_obstacle_failure_and_stale_scan_request_stop(self):
        self.activate();self.t['events'].append({'event':'FEEDBACK','code':306})
        self.assertTrue(self.evaluate()['stop_required']);self.assertEqual(self.evaluate()['stage'],'BLOCKED')
        self.t['events']=[];self.s['scan']['received_unix']=90
        self.assertTrue(self.evaluate()['stop_required'])
    def test_detour_has_grace_but_stall_stops(self):
        self.activate();self.evaluate();self.s['pose']['x']=.3
        self.now=120;self.refresh();self.assertFalse(self.evaluate()['stop_required'])
        self.now=131;self.refresh();self.assertTrue(self.evaluate()['stop_required'])
    def test_manual_new_preview_does_not_inherit_old_cancel(self):
        self.t.update(goal=self.goal,session_started_unix=99,events=[{'event':'STOP_RESULT','result':0,'code':0},{'event':'RESULT','status':6,'result':5}])
        self.assertEqual(self.evaluate()['stage'],'READY')
    def test_relocalization_pause_is_not_treated_as_lost_localization(self):
        self.activate();self.t['events']=[{'event':'RELOCALIZATION_BEGIN'}];self.s['task']['status']=101;self.s['pose']['received_unix']=90
        self.assertEqual(self.evaluate()['stage'],'RELOCALIZING');self.assertFalse(self.evaluate()['stop_required'])
        self.t['events'].append({'event':'RELOCALIZATION_SUCCESS'})
        self.assertEqual(self.evaluate()['stage'],'REPLANNING')

if __name__=='__main__':unittest.main()
