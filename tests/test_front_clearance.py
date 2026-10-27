import unittest
from manual_policy import allowed,front_clearance,limited_velocity,FRONT_EXTENT_M
import test_manual_policy as policy_tests

class FrontClearanceTests(unittest.TestCase):
    def health(self,clearance):
        h=policy_tests.ManualPolicyTests().supervised_health()
        h['depth_min']=FRONT_EXTENT_M+clearance
        return h
    def test_boundary_for_three_forward_directions(self):
        for direction in ('forward','forward_left','forward_right'):
            for distance in (.1,.199,.2):
                self.assertTrue(allowed(direction,self.health(distance),10.1))
            for distance in (.201,.25,.4,1):
                self.assertEqual('',allowed(direction,self.health(distance),10.1))
    def test_clearance_is_not_base_link_range(self):
        self.assertAlmostEqual(front_clearance(self.health(.2)),.2)
    def test_invalid_range_blocks(self):
        for value in (None,float('nan'),float('inf'),True,'1'):
            h=self.health(1);h['depth_min']=value
            self.assertTrue(allowed('forward',h,10.1))
    def test_approach_slows(self):
        self.assertAlmostEqual(limited_velocity('forward',None,self.health(.3),10.1)[0],.1)
        self.assertEqual(limited_velocity('forward',None,self.health(1),10.1),(.5,0.,0.))
    def test_boundary_and_stale_still_stop(self):
        self.assertEqual(limited_velocity('forward',None,self.health(.2),10.1),(0.,0.,0.))
        self.assertEqual(limited_velocity('forward',None,self.health(1),11),(0.,0.,0.))
    def test_arc_ratio_preserved_by_slowdown(self):
        vx,vy,w=limited_velocity('forward_left',None,self.health(.3),10.1)
        self.assertAlmostEqual(vx,.1);self.assertEqual(vy,0.)
        self.assertAlmostEqual(vx/w,.5/.6)
    def test_rearward_not_blocked_by_front_clearance(self):
        self.assertEqual(allowed('backward',self.health(.1),10.1),'')
