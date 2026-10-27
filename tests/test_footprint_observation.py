import unittest
from cloud_geometry import FootprintObservation
from test_unknown_goal import grid

class BodyObservationTests(unittest.TestCase):
    def test_clears_only_witnessed_body_unknown_not_occupied_or_surroundings(self):
        g=grid(False);g['data']=[-1]*len(g['data']);g['data'][15*40+10]=100
        obs=FootprintObservation();obs.observe({'x':1.05,'y':1.55,'yaw':0,'frame':'laser_odom'})
        result=obs.apply(g)
        self.assertEqual(result['data'][15*40+10],100)
        self.assertEqual(result['data'][15*40+11],0)
        self.assertEqual(result['data'][12*40+10],-1)
        self.assertEqual(g['data'][15*40+11],-1)
    def test_frame_and_jump_reset(self):
        obs=FootprintObservation();obs.observe({'x':0,'y':0,'yaw':0,'frame':'map'});self.assertFalse(obs.poses)
        obs.observe({'x':0,'y':0,'yaw':0,'frame':'laser_odom'});obs.observe({'x':2,'y':0,'yaw':0,'frame':'laser_odom'})
        self.assertEqual(len(obs.poses),1)
