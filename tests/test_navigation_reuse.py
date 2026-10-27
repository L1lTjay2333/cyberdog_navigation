import ast
import math
import time
import types
import unittest
from pathlib import Path

class ReuseTests(unittest.TestCase):
    def run_reuse(self,status=8,shift=0,age=0):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text())
        fn=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('reuse_active_localization','check_localization_jump')]
        item=types.SimpleNamespace;events=[];calls=[];stamp=time.time()-age
        transform=item(header=item(stamp=item(sec=int(stamp),nanosec=int((stamp%1)*1e9))),
            transform=item(translation=item(x=1+shift,y=2),rotation=item(x=0,y=0,z=0,w=1)))
        def call(*args):calls.append(args[0]);return item(status=status,code=0)
        env={'REQUEST':{'map_name':'test','expected_start':{'frame':'map','x':1,'y':2,'yaw':0}},
            'operator_ready':lambda:None,'assert_active_map':lambda *_:None,'call':call,
            'AlgoTaskStatus':item(Request=lambda:None),'buffer':item(lookup_transform=lambda *_:transform),
            'node':None,'rclpy':item(time=item(Time=lambda:None),spin_once=lambda *_,**__:None),'time':time,'math':math,'emit':lambda **row:events.append(row)}
        exec(compile(ast.Module(body=fn,type_ignores=[]),'reuse','exec'),env)
        return env,calls,events
    def test_valid_localization_reused_without_reset(self):
        env,calls,events=self.run_reuse();env['reuse_active_localization']()
        self.assertEqual(calls,['algo_task_status']);self.assertEqual(events[-1]['event'],'LOCALIZATION_REUSED')
    def test_other_task_rejected(self):
        env,_,_=self.run_reuse(status=1)
        with self.assertRaises(RuntimeError):env['reuse_active_localization']()
    def test_jump_rejected(self):
        env,_,_=self.run_reuse(shift=2)
        with self.assertRaises(RuntimeError):env['reuse_active_localization']()
    def test_stale_rejected(self):
        env,_,_=self.run_reuse(age=5)
        with self.assertRaises(RuntimeError):env['reuse_active_localization']()
