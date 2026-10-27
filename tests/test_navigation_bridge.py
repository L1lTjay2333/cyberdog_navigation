import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch,MagicMock
from navigation_bridge import NavigationBridge

class BridgeTests(unittest.TestCase):
    def test_idle_recovery_clears_failed_goal(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,True));bridge.events=[{'event':'ERROR'}];bridge.goal={'x':1}
            bridge.recover(timeout=0)
            self.assertEqual(bridge.events,[]);self.assertIsNone(bridge.goal)
            self.assertFalse(bridge.active)
    def test_recovery_does_not_hide_unconfirmed_motion(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,True));bridge.active=True;bridge.finished.clear()
            bridge.events=[{'event':'SEND_ONCE'}]
            with self.assertRaisesRegex(ValueError,'停车尚未确认'):bridge.recover(timeout=0)
            self.assertTrue(bridge.active);self.assertTrue(bridge.cancel_event.is_set())
    def test_close_failure_still_releases_session(self):
        with tempfile.TemporaryDirectory() as d:
            root=self.root(d,True);(root/'tools').mkdir();(root/'evidence').mkdir()
            for name in ('camera_handoff.py','map_inventory.py','stock_navigation_session.py','robot_heartbeat.py'):
                (root/'tools'/name).write_text('')
            process=MagicMock();process.stdout=[];process.stdin.close.side_effect=BrokenPipeError()
            bridge=NavigationBridge(root);bridge.active=True;bridge.finished.clear()
            with patch('navigation_bridge.subprocess.Popen',return_value=process),patch('navigation_bridge.threading.Thread'):
                bridge.run({'x':1,'y':0,'frame':'map'})
            self.assertFalse(bridge.active);self.assertTrue(bridge.finished.is_set());self.assertIsNone(bridge.process)
    def test_startup_does_not_latch_temporary_browser_delay_as_cancel(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,False));bridge.transport_started=time.monotonic();bridge.last_browser_poll-=4
            self.assertFalse(bridge.heartbeat_message()['cancel']);self.assertFalse(bridge.heartbeat_message()['operator_live'])
            bridge.transport_started-=16;self.assertTrue(bridge.heartbeat_message()['cancel'])
            bridge.transport_started=time.monotonic();bridge.cancel();self.assertTrue(bridge.heartbeat_message()['cancel'])
    def root(self,directory,enabled):
        p=Path(directory);(p/'config').mkdir();(p/'config/scope.json').write_text(json.dumps({'motion_enabled':enabled}));return p
    def test_locked_never_starts_transport(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,False))
            with patch('navigation_bridge.subprocess.Popen') as transport:
                with self.assertRaises(ValueError):bridge.start({'x':0,'y':0,'frame':'map'})
                transport.assert_not_called();self.assertFalse(bridge.active)
    def test_busy_rejects_second_goal(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,True));bridge.active=True
            with patch('navigation_bridge.subprocess.Popen') as transport:
                with self.assertRaises(ValueError):bridge.start({'x':0,'y':0,'frame':'map'})
                transport.assert_not_called()
    def test_browser_disconnect_requests_stop(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,False))
            self.assertFalse(bridge.heartbeat_message()['cancel'])
            bridge.last_browser_poll-=4
            self.assertTrue(bridge.heartbeat_message()['cancel'])
            bridge.operator_ping()
            self.assertFalse(bridge.heartbeat_message()['cancel'])
            bridge.cancel()
            self.assertTrue(bridge.heartbeat_message()['cancel'])
    def test_refresh_only_can_start_while_motion_locked(self):
        with tempfile.TemporaryDirectory() as d:
            bridge=NavigationBridge(self.root(d,False))
            with patch('navigation_bridge.threading.Thread') as thread:
                bridge.start_localization()
                self.assertEqual(thread.call_args.kwargs['args'],({'action':'localize'},))
                self.assertIsNone(bridge.goal)
                self.assertFalse(bridge.enabled())
