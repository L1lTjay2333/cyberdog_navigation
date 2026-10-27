import unittest
from unittest.mock import patch
import time
from manual_bridge import ManualBridge

class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.bridge=ManualBridge('.')
        self.bridge.owner='owner-one';self.bridge.owner_seen=10
        self.bridge.enabled=lambda:False
    def test_abandoned_page_expires(self):
        with patch.object(self.bridge,'disconnect') as stop:
            self.assertTrue(self.bridge.expire_owner(13.1));stop.assert_called_once()
    def test_live_page_not_expired(self):
        self.assertFalse(self.bridge.expire_owner(12.9))
    def test_other_page_cannot_renew_owner(self):
        self.bridge.report('different-page')
        self.assertEqual(self.bridge.owner_seen,10)
    def test_own_page_renews(self):
        self.bridge.report('owner-one')
        self.assertGreater(self.bridge.owner_seen,10)
    def test_accepted_direction_renews_owner_without_state_poll(self):
        self.bridge.state={'connected':True}
        self.bridge.last_state=time.monotonic()
        self.bridge.control({'owner':'owner-one','sequence':1,
                             'direction':'forward','armed':False})
        renewed=self.bridge.owner_seen
        self.assertGreater(renewed,10)
        with patch.object(self.bridge,'disconnect') as stop:
            self.assertFalse(self.bridge.expire_owner(renewed+2.9))
            stop.assert_not_called()
    def test_rejected_direction_does_not_renew_other_owner(self):
        with self.assertRaisesRegex(ValueError,'请先在此网页连接遥控'):
            self.bridge.control({'owner':'other-page','sequence':1,
                                 'direction':'forward','armed':False})
        self.assertEqual(self.bridge.owner_seen,10)
    def test_disconnection_revokes_and_disarms(self):
        self.bridge.command={'direction':'forward','armed':True}
        self.bridge.disconnect()
        self.assertIsNone(self.bridge.owner)
        self.assertEqual(self.bridge.command,{'direction':'stop','armed':False})

if __name__=='__main__':unittest.main()
