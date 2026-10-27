"""The manual recovery button must not clear or reset an active robot goal."""
import subprocess
import time
import unittest
from unittest.mock import patch

from stale_task_reset import reset_stale_navigation


class StaleTaskResetTests(unittest.TestCase):
    def setUp(self):
        self.now=time.time()
        self.snapshot={'connection':'connected','local_received_unix':self.now,
                       'emitted_unix':self.now,
                       'task':{'status':1,'received_unix':self.now}}
        self.nav={'active':False}
        self.mapping={'active':False}
        self.exploring={'active':False}

    def run_reset(self,confirmed=True):
        return reset_stale_navigation(__import__('pathlib').Path(__file__).resolve().parents[1],
            self.snapshot,self.nav,self.mapping,False,self.exploring,confirmed)

    def test_requires_stopped_confirmation(self):
        with patch('stale_task_reset.subprocess.run') as run:
            with self.assertRaisesRegex(ValueError,'停稳'):self.run_reset(False)
            run.assert_not_called()

    def test_rejects_other_task_states_or_active_session(self):
        with patch('stale_task_reset.subprocess.run') as run:
            self.snapshot['task']['status']=8
            with self.assertRaises(ValueError):self.run_reset()
            self.snapshot['task']['status']=1
            self.nav['active']=True
            with self.assertRaises(ValueError):self.run_reset()
            run.assert_not_called()

    def test_requires_live_robot_feedback(self):
        self.snapshot['local_received_unix']-=5
        with patch('stale_task_reset.subprocess.run') as run:
            with self.assertRaises(ValueError):self.run_reset()
            run.assert_not_called()

    def test_only_reports_success_after_robot_confirmation(self):
        with patch('stale_task_reset.subprocess.run',return_value=subprocess.CompletedProcess([],0,
             '{"event":"COMPLETED_NAVIGATION_RESET","result":0,"code":0}\n','')):
            self.assertTrue(self.run_reset()['ok'])
        with patch('stale_task_reset.subprocess.run',return_value=subprocess.CompletedProcess([],1,'','active action')):
            with self.assertRaisesRegex(ValueError,'未确认'):self.run_reset()


if __name__=='__main__':unittest.main()
