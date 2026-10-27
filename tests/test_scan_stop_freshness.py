import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as S

class FreshnessTests(unittest.TestCase):
    def env(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text(encoding='utf-8'))
        functions=[f for f in tree.body if isinstance(f,ast.FunctionDef) and f.name in ('navigation_scan_receive','stop_odom_receive')]
        clock=[100.];env={'math':math,'time':S(time=lambda:clock[0],monotonic=lambda:clock[0]),'observations':{'scan':0}}
        exec(compile(ast.Module(body=functions,type_ignores=[]),'freshness','exec'),env)
        return env,clock
    def msg(self,stamp=100,speed=0):
        return S(header=S(stamp=S(sec=stamp,nanosec=0)),twist=S(twist=S(linear=S(x=speed,y=0,z=0),angular=S(x=0,y=0,z=0))))
    def test_old_scan_received_now_is_not_fresh(self):
        e,_=self.env();e['navigation_scan_receive'](self.msg(95));self.assertEqual(e['observations']['scan'],0)
        e['navigation_scan_receive'](self.msg());self.assertEqual(e['observations']['scan'],100)
    def test_motion_and_missing_samples_break_stationary_window(self):
        e,t=self.env();e['stop_odom_receive'](self.msg());self.assertEqual(e['observations']['stationary_since'],100)
        t[0]=100.1;e['stop_odom_receive'](self.msg(speed=.1));self.assertIsNone(e['observations']['stationary_since'])
        t[0]=101;e['stop_odom_receive'](self.msg(101));self.assertEqual(e['observations']['stationary_since'],101)
        t[0]=103;e['stop_odom_receive'](self.msg(103));self.assertEqual(e['observations']['stationary_since'],103)
    def test_stale_odometry_cannot_verify_stop(self):
        e,_=self.env();e['stop_odom_receive'](self.msg(95));self.assertIsNone(e['observations']['stationary_since'])
