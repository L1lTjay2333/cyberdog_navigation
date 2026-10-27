import unittest
from motion_trail import MotionTrail
from localization_validity import LocalizationValidity
from telemetry_time import sensor_age

class LiveTrajectoryTests(unittest.TestCase):
    def snapshot(self,x=0,t=100):
        return {'connection':'connected','started_unix':1,'local_received_unix':t+10,'emitted_unix':t,
                'task':{'status':5},'map':{'frame':'laser_odom','width':10},
                'pose':{'frame':'laser_odom','x':x,'y':0.,'yaw':0.,'received_unix':t}}

    def test_map_growth_and_page_reads_preserve_world_trajectory(self):
        trail=MotionTrail();s=self.snapshot();trail.update(s,'scan',110)
        s=self.snapshot(.2,100.25);s['map']['width']=20
        r=trail.update(s,'scan',110.25)
        self.assertEqual(len(r['points']),2);self.assertAlmostEqual(r['distance_m'],.2)
        self.assertEqual(len(trail.update(s,'scan',110.3)['points']),2)

    def test_jump_and_new_scan_do_not_draw_false_connecting_line(self):
        trail=MotionTrail();trail.update(self.snapshot(),'scan',110)
        r=trail.update(self.snapshot(5,100.25),'scan',110.25)
        self.assertTrue(r['points'][-1]['break']);self.assertEqual(r['distance_m'],0)
        r=trail.update(self.snapshot(5,100.5),'next_scan',110.5)
        self.assertEqual(len(r['points']),1)

    def test_old_pose_not_recorded_despite_new_receipt(self):
        s=self.snapshot();s['pose']['received_unix']=90
        self.assertEqual(MotionTrail().update(s,'scan',110)['points'],[])

    def test_pc_clock_offset_does_not_prevent_completed_localization(self):
        for offset in (-30,2.3,30):
            guard=LocalizationValidity();guard.invalid_since=100+offset
            s=self.snapshot(t=101);s['local_received_unix']=101+offset
            s['pose']['frame']='map';s['map']['frame']='map';s['task']['status']=8
            e={'event':'POSITION_REFRESHED','unix':100.9,'local_received_unix':100.9+offset,'x':0.,'y':0.,'yaw':0.}
            self.assertFalse(guard.update(s,{'events':[e]},'room',101+offset)['required'])
            s['pose']['received_unix']=90
            self.assertGreater(sensor_age(s,90,101+offset),2)

    def test_old_result_does_not_validate_after_moving(self):
        s=self.snapshot(t=101);s['pose']['frame']='map';s['map']['frame']='map';s['task']['status']=8
        guard=LocalizationValidity();guard.invalid_since=110
        e={'event':'POSITION_REFRESHED','unix':99,'local_received_unix':109,'x':0.,'y':0.,'yaw':0.}
        self.assertTrue(guard.update(s,{'events':[e]},'room',111)['required'])
