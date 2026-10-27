import unittest
from unittest.mock import patch
import map_viewer as viewer


class BatteryFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now=100
        self.snapshot={'connection':'connected','local_received_unix':100,'emitted_unix':100,
            'battery':{'percent':45,'wired_charging':False,'wireless_charging':False,'received_unix':100}}
        self.manual={'connected':False,'battery':{'percent':40,'wired_charging':True,'received_unix':90}}

    def test_disconnected_remote_session_uses_passive_unplug_update(self):
        battery,source=viewer.current_battery(self.manual,self.snapshot,self.now)
        self.assertFalse(battery['wired_charging']);self.assertEqual(source,'passive_bms')

    def test_newest_bms_wins_even_with_active_manual_session(self):
        self.manual['connected']=True;self.manual['battery']['received_unix']=99
        battery,_=viewer.current_battery(self.manual,self.snapshot,self.now)
        self.assertFalse(battery['wired_charging'])
        self.snapshot['battery']['received_unix']=98
        battery,source=viewer.current_battery(self.manual,self.snapshot,self.now)
        self.assertTrue(battery['wired_charging']);self.assertEqual(source,'manual_bms')

    def test_disconnected_passive_stream_is_not_live_charging_feedback(self):
        self.snapshot['connection']='disconnected'
        self.assertEqual(viewer.current_battery(self.manual,self.snapshot,self.now),(None,'unavailable'))

    def test_stale_battery_returns_unknown_not_charging(self):
        self.snapshot['battery']['received_unix']=90
        self.assertEqual(viewer.current_battery(self.manual,self.snapshot,self.now),(None,'unavailable'))

    def test_stale_packet_with_recent_battery_is_rejected(self):
        self.snapshot['emitted_unix']=90
        self.assertEqual(viewer.current_battery(self.manual,self.snapshot,self.now),(None,'unavailable'))

    def test_invalid_battery_is_rejected(self):
        self.snapshot['battery']['percent']=float('nan')
        self.assertEqual(viewer.current_battery(self.manual,self.snapshot,self.now),(None,'unavailable'))

    def test_zero_percentage_remains_valid_feedback(self):
        self.snapshot['battery']['percent']=0
        battery,_=viewer.current_battery(self.manual,self.snapshot,self.now)
        self.assertEqual(battery['percent'],0)

    def test_robot_clock_offset_does_not_expire_live_battery(self):
        self.snapshot['emitted_unix']=95
        self.snapshot['battery']['received_unix']=94.8
        battery,source=viewer.current_battery(self.manual,self.snapshot,self.now)
        self.assertEqual(source,'passive_bms')
        self.assertAlmostEqual(battery['received_unix'],99.8)

    def test_offline_record_cannot_claim_unplug_detected(self):
        with patch.object(viewer,'OFFLINE',True):
            self.assertEqual(viewer.current_battery(self.manual,self.snapshot,self.now),(None,'unavailable'))
