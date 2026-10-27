import ast
import math
from pathlib import Path
from types import SimpleNamespace as Item
import unittest


class SettleTests(unittest.TestCase):
    def run_wait(self, jump=False, frozen=False, drifting=False):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text())
        nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='wait_for_stable_localization']
        elapsed=[0.];events=[]
        def spin(*args,**kwargs):elapsed[0]+=.1
        def lookup(*args):
            stamp=100 if frozen else 100+elapsed[0]
            x=elapsed[0] if drifting else (2. if jump and elapsed[0]>=1 else 0.)
            return Item(header=Item(stamp=Item(sec=int(stamp),nanosec=int((stamp%1)*1e9))),
                        transform=Item(translation=Item(x=x,y=0.),rotation=Item(x=0.,y=0.,z=0.,w=1.)))
        env={'math':math,'time':Item(monotonic=lambda:elapsed[0],time=lambda:100+elapsed[0]),
             'rclpy':Item(spin_once=spin,time=Item(Time=lambda:None)),
             'buffer':Item(lookup_transform=lookup),'node':None,'operator_ready':lambda:None,
             'emit':lambda **v:events.append(v)}
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'settle','exec'),env)
        return env['wait_for_stable_localization'](),elapsed[0]

    def test_stable_requires_multiple_fresh_frames(self):
        _,elapsed=self.run_wait();self.assertGreaterEqual(elapsed,2.)

    def test_initial_identity_then_jump_restarts_window(self):
        transform,elapsed=self.run_wait(jump=True)
        self.assertEqual(transform.transform.translation.x,2.);self.assertGreaterEqual(elapsed,3.)

    def test_frozen_transform_is_not_stability(self):
        with self.assertRaises(RuntimeError):self.run_wait(frozen=True)

    def test_continuous_drift_fails(self):
        with self.assertRaises(RuntimeError):self.run_wait(drifting=True)
