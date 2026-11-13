import math
import unittest
from pathlib import Path
from types import SimpleNamespace as N

def transform(frame='laser_odom',xyz=(0,0,0),q=(0,0,0,1),ts=10):
    return N(header=N(frame_id=frame,stamp=N(sec=ts,nanosec=0)),child_frame_id='',
        transform=N(translation=N(**dict(zip(('x','y','z'),xyz))),rotation=N(**dict(zip(('x','y','z','w'),q)))))

class Poses(unittest.TestCase):
    def setUp(self):
        source=Path('tools/reconstruction_pose.py').read_text(encoding='utf8')
        source='\n'.join(line for line in source.splitlines() if not line.startswith('from '))
        env={'TransformStamped':transform}
        exec(source,env)
        self.pose=env['ReconstructionPose'].__new__(env['ReconstructionPose'])
        self.pose.frame=None
        self.stamp=N(nanoseconds=10000000000)
    def test_body_rotation_applies_to_camera_offset(self):
        base=transform(xyz=(1,2,3),q=(0,0,math.sqrt(.5),math.sqrt(.5)))
        self.pose.mapping=N(lookup_transform=lambda *args:base)
        self.pose.buffer=N(lookup_transform=lambda *args:transform('base_link',(1,0,0)))
        result=self.pose.lookup('camera',self.stamp)
        p=result.transform.translation
        for actual,want in zip((p.x,p.y,p.z),(1,3,3)):self.assertAlmostEqual(actual,want)
        self.assertEqual(self.pose.frame,'laser_odom')
    def test_locked_mapping_cannot_fall_back_to_visual_odom(self):
        self.pose.frame='laser_odom'
        def absent(*args):raise ValueError('missing mapping')
        self.pose.mapping=N(lookup_transform=absent)
        self.pose.buffer=N(lookup_transform=lambda *args:transform('vodom'))
        with self.assertRaisesRegex(ValueError,'missing mapping'):self.pose.lookup('camera',self.stamp)
    def test_visual_fallback_locks_coordinate_frame(self):
        def absent(*args):raise ValueError('missing mapping')
        self.pose.mapping=N(lookup_transform=absent)
        self.pose.buffer=N(lookup_transform=lambda *args:transform('vodom'))
        self.pose.lookup('camera',self.stamp)
        self.assertEqual(self.pose.frame,'vodom')
    def test_stale_transform_not_accepted(self):
        self.pose.frame='vodom'
        self.pose.buffer=N(lookup_transform=lambda *args:transform('vodom',ts=9))
        with self.assertRaises(ValueError):self.pose.lookup('camera',self.stamp)
