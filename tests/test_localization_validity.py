import unittest
from localization_validity import LocalizationValidity
from navigation_decision import localization_problem


class ValidityTests(unittest.TestCase):
    def setUp(self):
        self.guard=LocalizationValidity();self.guard.invalid_since=90
        self.s={'connection':'connected','local_received_unix':100,'emitted_unix':100,'started_unix':80,
                'task':{'status':8,'received_unix':100},
                'pose':{'x':1.,'y':1.,'yaw':0.,'frame':'map','received_unix':100},
                'scan':{'received_unix':100,'finite_ranges':10},
                'map':{'frame':'map','width':100,'height':100,'resolution':.1,'origin':{'x':0,'y':0,'qz':0,'qw':1}},
                'battery':{'received_unix':100,'wired_charging':False},
                'odom':{'x':0.,'y':0.,'yaw':0.,'frame':'odom','received_unix':100}}
        self.t={'events':[]}
    def update(self,now=100,name='room'):
        self.s['local_received_unix']=now;self.s['emitted_unix']=now;self.s['pose']['received_unix']=now;self.s['battery']['received_unix']=now
        return self.guard.update(self.s,self.t,name,now)
    def verify(self):
        self.t['events']=[{'event':'POSITION_REFRESHED','unix':99,'x':1.,'y':1.,'yaw':0.}]
        self.assertFalse(self.update()['required'])
    def test_fresh_tf_and_status_eight_do_not_prove_relocation(self):
        r=self.update();self.assertTrue(r['required'])
        self.s['localization_validation']=r
        self.assertIsNotNone(localization_problem(self.s,100))
    def test_charge_invalidates_and_old_success_does_not_unlock(self):
        self.verify();self.s['battery']['wired_charging']=True
        self.assertTrue(self.update(101)['required'])
        self.s['battery']['wired_charging']=False
        self.assertTrue(self.update(102)['required'])
        self.t['events']=[{'event':'POSITION_REFRESHED','unix':103,'x':1.,'y':1.,'yaw':0.}]
        self.assertFalse(self.update(103)['required'])
    def test_stream_restart_invalidates(self):
        self.verify();self.s['started_unix']=101
        self.assertTrue(self.update(102)['required'])
    def test_new_map_invalidates_but_same_map_republication_does_not(self):
        self.verify();self.s['map']['received_unix']=101
        self.assertFalse(self.update(101)['required'])
        self.assertTrue(self.update(102,'new_room')['required'])
    def test_regular_motion_does_not_invalidate_but_odom_reset_does(self):
        self.verify();self.s['odom'].update(x=.05,yaw=.05,received_unix=100.1)
        self.assertFalse(self.update(100.1)['required'])
        self.s['odom'].update(x=2.,received_unix=100.2)
        self.assertTrue(self.update(100.2)['required'])
    def test_failed_or_wrong_heading_relocalization_does_not_unlock(self):
        self.t['events']=[{'event':'RELOCALIZATION_SUCCESS','unix':99}]
        self.assertTrue(self.update()['required'])
        self.t['events']=[{'event':'POSITION_REFRESHED','unix':99,'x':1.,'y':1.,'yaw':3.14}]
        self.assertTrue(self.update()['required'])

    def test_field_rejected_map_cannot_be_unlocked_by_native_success(self):
        self.guard=LocalizationValidity('room');self.guard.invalid_since=90
        self.t['events']=[{'event':'POSITION_REFRESHED','unix':99,'x':1.,'y':1.,'yaw':0.}]
        self.assertTrue(self.update()['rejected_map'])
        self.assertTrue(self.update()['required'])
        self.assertTrue(self.update(101,'new_room')['required'])
        self.t['events']=[{'event':'POSITION_REFRESHED','unix':102,'x':1.,'y':1.,'yaw':0.}]
        self.assertFalse(self.update(102,'new_room')['required'])
