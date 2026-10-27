import ast
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest

class MappingDisplayTransformTests(unittest.TestCase):
    def environment(self,status=5,x=1):
        source=Path(__file__).resolve().parents[1]/'tools/map_stream.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='display_transform']
        calls=[]
        def lookup(target,source,time):
            calls.append((target,source))
            return S(transform=S(translation=S(x=x,y=2),rotation=S(x=0,y=0,z=0,w=1)))
        env={'state':{'task':{'status':status},'raw_pose':{'frame':'laser_odom','x':1,'y':2,'yaw':0,'received_unix':100}},
             'tf_buffer':S(lookup_transform=lookup),'rclpy':S(time=S(Time=S(from_msg=lambda s:s))),'math':math}
        exec(compile(ast.Module(body=body,type_ignores=[]),str(source),'exec'),env)
        return env,calls
    def test_confirmed_mapping_alias_uses_vodom(self):
        env,calls=self.environment()
        env['display_transform']('laser_odom','laser_frame',S(sec=100,nanosec=0))
        self.assertEqual(calls,[('vodom','base_link'),('vodom','laser_frame')])
    def test_disagreement_not_relabelled(self):
        env,_=self.environment(x=5)
        with self.assertRaisesRegex(ValueError,'disagree'):
            env['display_transform']('laser_odom','laser_frame',S(sec=100,nanosec=0))
    def test_stale_mapping_pose_not_relabelled(self):
        env,_=self.environment()
        with self.assertRaisesRegex(ValueError,'timestamp'):
            env['display_transform']('laser_odom','laser_frame',S(sec=101,nanosec=0))
    def test_navigation_never_uses_mapping_alias(self):
        env,calls=self.environment(status=8)
        env['display_transform']('map','laser_frame',S(sec=100,nanosec=0))
        self.assertEqual(calls,[('map','laser_frame')])
