import ast
import math
import types
import unittest
from pathlib import Path


class InputWaitTests(unittest.TestCase):
    def environment(self,missing=False,localization_only=False,charging=False,percent=5):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text())
        functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('wait_for_navigation_inputs','battery_ready')]
        item=types.SimpleNamespace;elapsed=[0.];events=[]
        def spin(*_,**__):elapsed[0]+=.05
        transform=item(header=item(stamp=item(sec=100,nanosec=0)))
        def lookup(*_):
            if missing:raise RuntimeError('no TF')
            return transform
        env={'time':item(monotonic=lambda:elapsed[0],time=lambda:100),
             'rclpy':item(spin_once=spin,time=item(Time=lambda:None)), 'node':None,
             'buffer':item(lookup_transform=lookup),'operator_ready':lambda:None,
             'emit':lambda **row:events.append(row),
             'observations':{'map':None if localization_only else item(header=item(frame_id='map')),
                'scan':0.,'battery':item(batt_soc=percent,power_wired_charging=charging,power_wp_charging=False),
                'battery_received':0.}}
        exec(compile(ast.Module(body=functions,type_ignores=[]),'wait','exec'),env)
        return env,events,elapsed

    def test_ready_inputs_return_without_fixed_four_seconds(self):
        env,events,elapsed=self.environment();env['wait_for_navigation_inputs'](False,99)
        self.assertLess(elapsed[0],.2);self.assertEqual(events[-1]['event'],'INPUTS_READY')

    def test_localization_does_not_wait_for_unused_scan_map(self):
        env,_,elapsed=self.environment(localization_only=True);env['wait_for_navigation_inputs'](True,99)
        self.assertLess(elapsed[0],.2)

    def test_no_tf_still_times_out_instead_of_fake_readiness(self):
        env,events,elapsed=self.environment(missing=True)
        with self.assertRaises(RuntimeError):env['wait_for_navigation_inputs'](False,99)
        self.assertGreaterEqual(elapsed[0],4)
        self.assertEqual(events[-1]['missing'],['localization TF missing','scan stale','battery feedback stale'])

    def test_mapping_frame_cannot_erase_navigation_map(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text())
        functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='navigation_map_receive']
        env={'observations':{'map':None}}
        exec(compile(ast.Module(body=functions,type_ignores=[]),'receive','exec'),env)
        item=types.SimpleNamespace
        mapped=item(header=item(frame_id='map'))
        mapping=item(header=item(frame_id='laser_odom'))
        env['navigation_map_receive'](mapped);env['navigation_map_receive'](mapping)
        self.assertIs(env['observations']['map'],mapped)

    def test_missing_map_reports_specific_input(self):
        env,events,elapsed=self.environment();env['observations']['map']=None
        with self.assertRaisesRegex(RuntimeError,'navigation map missing'):
            env['wait_for_navigation_inputs'](False,99)
        self.assertIn('navigation map missing',events[-1]['missing'])

    def test_slow_subscription_startup_does_not_require_relocalization(self):
        env,events,elapsed=self.environment();grid=env['observations']['map']
        env['observations']['map']=None
        def delayed_spin(*args,**kwargs):
            elapsed[0]+=.05
            env['observations'].update(scan=elapsed[0],battery_received=elapsed[0])
            if elapsed[0]>=4.5:env['observations']['map']=grid
        env['rclpy'].spin_once=delayed_spin
        env['wait_for_navigation_inputs'](False,99)
        self.assertGreaterEqual(elapsed[0],4.5)
        self.assertEqual(events[-1]['event'],'INPUTS_READY')

    def test_low_percentage_accepted_by_robot_client(self):
        for percent in (0,5,19):
            env,_,_=self.environment(percent=percent);env['battery_ready']()

    def test_charging_or_stale_battery_rejected(self):
        env,_,elapsed=self.environment(charging=True)
        with self.assertRaisesRegex(RuntimeError,'charging'):env['battery_ready']()
        env,_,elapsed=self.environment();elapsed[0]=4
        with self.assertRaisesRegex(RuntimeError,'expired'):env['battery_ready']()
