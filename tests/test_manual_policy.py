import unittest
from manual_policy import allowed, DIRECTIONS

class ManualPolicyTests(unittest.TestCase):
    def healthy(self):
        return dict(status_time=10., task_time=10., scan_time=10., task=101,
                    status=dict(switch_status=0, ori_error=0, footpos_error=0, motor_error=[0]),
                    front_ranges=[2.], left_ranges=[1.], right_ranges=[1.])
    def test_clear_forward(self): self.assertEqual('',allowed('forward',self.healthy(),10.1))
    def test_stop_always_available(self): self.assertEqual('',allowed('stop',{},100.))
    def test_stale_scan(self): self.assertIn('过期',allowed('forward',self.healthy(),10.7))
    def test_near_return_blocks(self):
        h=self.healthy();h['front_ranges']=[.2];self.assertTrue(allowed('forward',h,10.1))
    def test_no_returns_blocks(self):
        h=self.healthy();h['front_ranges']=[];self.assertTrue(allowed('forward',h,10.1))
    def test_backward_locked(self): self.assertTrue(allowed('backward',self.healthy(),10.1))
    def test_lateral_is_not_rotation(self):
        self.assertGreater(DIRECTIONS['shift_left'][1],0)
        self.assertLess(DIRECTIONS['shift_right'][1],0)
        self.assertEqual(0,DIRECTIONS['shift_left'][2])
        self.assertEqual(0,DIRECTIONS['shift_right'][2])
    def test_lateral_locked_until_verified(self):
        self.assertTrue(allowed('shift_left',self.healthy(),10.1))
        self.assertTrue(allowed('shift_right',self.healthy(),10.1))
    def test_algorithm_owner_blocks(self):
        h=self.healthy();h['task']=1;self.assertTrue(allowed('left',h,10.1))
    def test_bad_motor_blocks(self):
        h=self.healthy();h['status']['motor_error']=[4];self.assertTrue(allowed('right',h,10.1))
    def depth_health(self):
        h=self.healthy();h.update(sensor_mode='depth_forward_trial',task=6,
            depth_time=10.,depth_valid=True,depth_min=1.5)
        return h
    def test_depth_forward_trial(self):self.assertEqual('',allowed('forward',self.depth_health(),10.1))
    def test_depth_stale_stops(self):self.assertTrue(allowed('forward',self.depth_health(),10.7))
    def test_depth_invalid_stops(self):
        h=self.depth_health();h['depth_valid']=False;self.assertTrue(allowed('forward',h,10.1))
    def test_invalid_recent_frame_not_misreported_as_timeout(self):
        h=self.depth_health();h.update(depth_time=5.,depth_frame_received=10.,depth_invalid_reason='前方覆盖不足')
        self.assertIn('覆盖不足',allowed('forward',h,10.1))
        self.assertNotIn('过期',allowed('forward',h,10.1))
    def test_depth_near_stops(self):
        h=self.depth_health();h['depth_min']=.45;self.assertTrue(allowed('forward',h,10.1))
    def test_depth_turning_not_enabled(self):self.assertTrue(allowed('left',self.depth_health(),10.1))
    def test_depth_active_navigation_blocks(self):
        h=self.depth_health();h['task']=1;self.assertTrue(allowed('forward',h,10.1))
    def supervised_health(self):
        h=self.depth_health();h.update(sensor_mode='supervised_four_way_trial',task=101)
        h['status']['motion_id']=112
        return h
    def test_supervised_wasd_policy(self):
        for direction in ('forward','backward','shift_left','shift_right'):
            self.assertEqual('',allowed(direction,self.supervised_health(),10.1))
    def test_supervised_bad_posture_blocks_all_directions(self):
        h=self.supervised_health();h['status']['motion_id']=101
        for direction in ('forward','backward','shift_left','shift_right'):
            self.assertTrue(allowed(direction,h,10.1))
    def test_recovered_standing_is_recognized(self):
        h=self.supervised_health();h['status']['motion_id']=111
        for direction in ('forward','backward','shift_left','shift_right'):
            self.assertEqual('',allowed(direction,h,10.1))
    def test_supervised_forward_still_checks_depth(self):
        h=self.supervised_health();h['depth_min']=.4
        self.assertTrue(allowed('forward',h,10.1))
    def test_supervised_all_directions_check_status_freshness(self):
        h=self.supervised_health();h['status_time']=0
        self.assertTrue(allowed('backward',h,10.1))
    def test_owned_mapping_manual_allowed_but_other_mapping_blocked(self):
        h=self.supervised_health();h['task']=5
        self.assertTrue(allowed('shift_left',h,10.1))
        h['auxiliary_task']=5
        self.assertEqual('',allowed('shift_left',h,10.1))
    def test_owned_localization_manual_allowed_but_navigation_blocked(self):
        h=self.supervised_health();h['task']=7;h['auxiliary_task']=7
        self.assertEqual('',allowed('forward',h,10.1))
        h['task']=1
        self.assertTrue(allowed('forward',h,10.1))

if __name__=='__main__': unittest.main()
