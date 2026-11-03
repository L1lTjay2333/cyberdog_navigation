import unittest
from manual_policy import allowed,limited_velocity,DIRECTIONS


class OverrideTests(unittest.TestCase):
    def health(self):
        return dict(sensor_mode='supervised_four_way_trial',manual_distance_override=True,
            auxiliary_task=5,task=5,task_time=10.,status_time=10.,depth_valid=False,
            depth_min=.1,status=dict(motion_id=112,switch_status=0,ori_error=0,footpos_error=0,motor_error=[]))

    def test_all_directions_ignore_obstacle_distance(self):
        for direction in DIRECTIONS:
            self.assertEqual(allowed(direction,self.health(),10.1),'')
        self.assertEqual(limited_velocity('forward',None,self.health(),10.1),DIRECTIONS['forward'])

    def test_faults_and_staleness_still_block(self):
        h=self.health();h['status']['motor_error']=[1]
        self.assertTrue(allowed('forward',h,10.1))
        self.assertTrue(allowed('forward',self.health(),13.))
        h=self.health();h['task']=1
        self.assertTrue(allowed('forward',h,10.1))

    def test_without_override_original_distance_guard_remains(self):
        h=self.health();h['manual_distance_override']=False
        self.assertTrue(allowed('forward',h,10.1))
