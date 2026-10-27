"""Exercise the real passive-stream selector without importing ROS."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as Item
import unittest

class DisplayPoseTests(unittest.TestCase):
    def test_saved_map_remains_visible_when_idle(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/map_stream.py').read_text(encoding='utf-8'))
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='select_display_grid')
        env={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'grid selector','exec'),env)
        grids={'map':{'frame':'map','width':10},'laser_odom':{'frame':'laser_odom','width':100}}
        self.assertIs(env['select_display_grid'](grids,5),grids['laser_odom'])
        self.assertIs(env['select_display_grid'](grids,101),grids['map'])
        self.assertIs(env['select_display_grid'](grids,8),grids['map'])
        grids.pop('map');self.assertIsNone(env['select_display_grid'](grids,8))
    def select(self,status=8,tf_age=0,missing=False,previous=None):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/map_stream.py').read_text(encoding='utf-8'))
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='select_display_pose')
        def transform(*_):
            if missing:raise RuntimeError('No TF')
            return Item(header=Item(stamp=Item(sec=100-tf_age,nanosec=0)),
                        transform=Item(translation=Item(x=1,y=2),rotation=Item(x=0,y=0,z=0,w=1)))
        env={'math':math,'tf_buffer':Item(lookup_transform=transform),'rclpy':Item(time=Item(Time=lambda:None))}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'pose selector','exec'),env)
        raw={'x':9,'y':9,'yaw':0,'frame':'laser_odom','received_unix':100}
        return env['select_display_pose'](status,{'frame':'laser_odom' if status==5 else 'map'},raw,previous,100)
    def test_navigation_uses_verified_tf_not_mapping_label(self):
        pose=self.select();self.assertEqual(pose['frame'],'map');self.assertEqual(pose['x'],1)
    def test_no_transform_does_not_relabel_raw_pose(self):
        self.assertIsNone(self.select(missing=True))
    def test_old_matched_pose_keeps_real_timestamp(self):
        old={'frame':'map','x':1,'y':2,'yaw':0,'received_unix':90}
        self.assertEqual(self.select(tf_age=5,previous=old)['received_unix'],90)
    def test_mapping_uses_actual_raw_pose(self):
        pose=self.select(status=5,missing=True);self.assertEqual(pose['x'],9);self.assertEqual(pose['frame'],'laser_odom')
    def test_localization_transition_has_no_old_marker(self):
        self.assertIsNone(self.select(status=103,previous={'frame':'map','received_unix':100}))
