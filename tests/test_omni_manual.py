import math
import unittest
from manual_policy import checked_axes,axes_direction,command_velocity,allowed,DIRECTIONS,ARC_YAW_RATE
import test_manual_policy as policy_tests

class OmniTests(unittest.TestCase):
    def test_requested_speeds(self):
        self.assertEqual(DIRECTIONS['forward'],(.5,0,0))
        self.assertEqual(DIRECTIONS['shift_left'],(0,.3,0))
    def test_rotation_axes_and_buttons_agree(self):
        for y,direction in ((1,'left'),(-1,'right')):
            self.assertEqual(axes_direction([0,y]),direction)
            self.assertEqual(command_velocity(direction,[0,y]),DIRECTIONS[direction])
            self.assertEqual(DIRECTIONS[direction],(0,0,ARC_YAW_RATE*y))
    def test_rotating_during_owned_mapping_without_forward_depth(self):
        h=policy_tests.ManualPolicyTests().supervised_health()
        h.update(task=5,auxiliary_task=5,depth_valid=False,depth_time=0)
        for direction in ('left','right','shift_left','shift_right'):
            self.assertEqual(allowed(direction,h,10.1),'')
        self.assertTrue(allowed('forward',h,10.1))
        h['auxiliary_task']=None
        self.assertTrue(allowed('left',h,10.1))
    def test_eight_directions(self):
        for x,y,name in ((1,1,'forward_left'),(1,-1,'forward_right'),(-1,1,'backward_left'),(-1,-1,'backward_right')):
            axes=[x/math.sqrt(2),y/math.sqrt(2)]
            self.assertEqual(axes_direction(axes),name)
            vx,vy,w=command_velocity(name,axes)
            self.assertAlmostEqual((vx/.5)**2+(w/ARC_YAW_RATE)**2,1);self.assertEqual(vy,0)
            self.assertEqual(w>0,x*y>0)
    def test_continuous_stick(self):
        self.assertEqual(command_velocity('forward_left',[.4,.2]),(.2,0,ARC_YAW_RATE*.2))
        self.assertEqual(command_velocity('backward_left',[-.4,.2]),(-.2,0,-ARC_YAW_RATE*.2))
        self.assertEqual(command_velocity('forward_right',[.4,-.2]),(.2,0,-ARC_YAW_RATE*.2))
        self.assertEqual(command_velocity('backward_right',[-.4,-.2]),(-.2,0,ARC_YAW_RATE*.2))
    def test_stronger_arc_without_more_translation(self):
        vx,vy,yaw=command_velocity('forward_left',[1/math.sqrt(2)]*2)
        self.assertAlmostEqual(vx,.5/math.sqrt(2))
        self.assertEqual(vy,0)
        self.assertAlmostEqual(yaw,.60/math.sqrt(2))
        self.assertAlmostEqual(abs(vx/yaw),.5/.60)
    def test_four_arcs_bend_toward_requested_travel_side(self):
        for name,left,forward in (('forward_left',True,True),('forward_right',False,True),('backward_left',True,False),('backward_right',False,False)):
            vx,vy,w=command_velocity(name)
            self.assertEqual(vy,0)
            self.assertEqual(vx>0,forward)
            # Constant-twist planar model, 1 second; not physical robot evidence.
            delta_y=vx/w*(1-math.cos(w))
            self.assertEqual(delta_y>0,left)
    def test_invalid_vectors_rejected(self):
        for axes in ([1,1],[float('nan'),0],[True,0],[1],None):
            with self.assertRaises(ValueError):checked_axes(axes)
    def test_direction_mismatch_rejected(self):
        with self.assertRaises(ValueError):command_velocity('forward',[0,1])
    def test_front_diagonal_checks_obstacle(self):
        h=policy_tests.ManualPolicyTests().supervised_health()
        self.assertEqual(allowed('forward_left',h,10.1),'')
        h['depth_min']=.4;self.assertTrue(allowed('forward_left',h,10.1))
    def test_charging_blocks_motion_but_not_stop(self):
        h=policy_tests.ManualPolicyTests().supervised_health();h['battery']={'wired_charging':True}
        self.assertTrue(allowed('forward',h,10.1));self.assertEqual(allowed('stop',h,10.1),'')
