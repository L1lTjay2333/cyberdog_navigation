import math
import unittest
from map_coordinates import project_pose

class MapProjectionTests(unittest.TestCase):
    def grid(self, angle=0):
        return {'width':10,'height':10,'resolution':0.5,'frame':'laser_odom',
                'origin':{'x':1,'y':2,'qz':math.sin(angle/2),'qw':math.cos(angle/2)}}
    def test_origin_translation_and_y_flip(self):
        p=project_pose(self.grid(),{'x':2,'y':3,'yaw':math.pi/2,'frame':'laser_odom'},100,100)
        self.assertEqual((p['x'],p['y']),(20,80))
        self.assertAlmostEqual(p['angle'],-math.pi/2)
    def test_rotated_origin(self):
        p=project_pose(self.grid(math.pi/2),{'x':0,'y':3,'yaw':math.pi/2,'frame':'laser_odom'},100,100)
        self.assertAlmostEqual(p['x'],20)
        self.assertAlmostEqual(p['y'],80)
        self.assertAlmostEqual(p['angle'],0)
    def test_mismatch_and_outside_rejected(self):
        for pose in [{'x':2,'y':3,'yaw':0,'frame':'odom'},{'x':-1,'y':3,'yaw':0,'frame':'laser_odom'}]:
            with self.assertRaises(ValueError):project_pose(self.grid(),pose)
