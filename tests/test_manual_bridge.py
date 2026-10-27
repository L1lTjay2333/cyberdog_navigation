import json
import tempfile
import time
import unittest
from pathlib import Path
from manual_bridge import ManualBridge

class ManualBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        (self.root/'config').mkdir()
        (self.root/'config/scope.json').write_text(json.dumps({'manual_motion_enabled':False}))
        self.b=ManualBridge(self.root);self.b.owner='test_browser';self.b.state={'connected':True}
        self.b.last_state=time.monotonic()
    def tearDown(self):self.tmp.cleanup()
    def command(self,seq=1,**kw):
        return dict(owner='test_browser',sequence=seq,direction='forward',armed=False,**kw)
    def test_locked_command_is_dryrun(self):
        self.b.control(self.command());self.assertFalse(self.b.command['armed'])
    def test_locked_arm_rejected(self):
        c=self.command();c['armed']=True
        with self.assertRaises(ValueError):self.b.control(c)
    def test_stop_wins_out_of_order(self):
        self.b.control(dict(owner='test_browser',sequence=2,direction='stop'))
        self.b.control(self.command(1));self.assertEqual('stop',self.b.command['direction'])
    def test_other_tab_rejected(self):
        c=self.command();c['owner']='other_browser'
        with self.assertRaises(ValueError):self.b.control(c)
    def test_administrative_stop_revokes(self):
        self.b.stop()
        with self.assertRaises(ValueError):self.b.control(self.command())
    def test_reports_same_tab_ownership(self):
        self.assertTrue(self.b.report('test_browser')['owns_session'])
        self.assertFalse(self.b.report('another_browser')['owns_session'])
    def test_expired_session_does_not_keep_old_charging_state(self):
        self.b.state.update(battery={'percent':50,'wired_charging':True},
            motion_status={'switch_status':14},reason='机器狗正在充电',
            direction_blocks={'forward':'机器狗正在充电'})
        self.b.last_state-=2
        report=self.b.report()
        self.assertIsNone(report['battery']);self.assertIsNone(report['motion_status'])
        self.assertNotIn('充电',report['reason'])
        self.assertIn('未就绪',report['direction_blocks']['forward'])
