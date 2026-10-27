import unittest
from manual_policy import posture_target

class PostureTests(unittest.TestCase):
    def health(self,motion):
        return dict(status_time=10,task_time=10,task=101,status=dict(motion_id=motion,switch_status=0,ori_error=0,footpos_error=0,motor_error=[0]))
    def test_stand_to_lie(self):
        for m in (111,112):self.assertEqual(posture_target(self.health(m),10)[0],101)
    def test_lie_to_stand(self):
        for m in (101,102,143):self.assertEqual(posture_target(self.health(m),10)[0],111)
    def test_unknown_rejected(self):
        with self.assertRaises(ValueError):posture_target(self.health(303),10)
    def test_stale_rejected(self):
        with self.assertRaises(ValueError):posture_target(self.health(101),12)
    def test_switch_rejected(self):
        h=self.health(101);h['status']['switch_status']=1
        with self.assertRaises(ValueError):posture_target(h,10)
    def test_charge_status_explained(self):
        h=self.health(101);h['status']['switch_status']=14
        with self.assertRaisesRegex(ValueError,'充电状态'):posture_target(h,10)
    def test_charging_rejected(self):
        h=self.health(101);h['battery']={'wired_charging':True}
        with self.assertRaises(ValueError):posture_target(h,10)
