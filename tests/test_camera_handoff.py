import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from camera_handoff import release_competing_camera


class CameraHandoffTests(unittest.TestCase):
    def run_handoff(self, target='camera_align', states=None, reject=False):
        states = dict(states or {'camera': 3, 'camera_align': 1})
        calls = []
        def state(name):
            calls.append(('get', name))
            value = states[name]
            if value is None: raise RuntimeError('service timeout')
            return value
        def transition(name, value):
            calls.append(('change', name, value))
            if not reject: states[name] = {4: 2, 2: 1}[value]
        try:
            release_competing_camera(target, state, transition, lambda:None,
                                     lambda **kw:calls.append(('event',kw)))
            error = None
        except RuntimeError as exc: error = str(exc)
        return calls, states, error

    def test_release_normal_capture_before_navigation(self):
        calls, states, error = self.run_handoff()
        self.assertIsNone(error)
        self.assertEqual(states['camera'],1)
        self.assertEqual([x for x in calls if x[0]=='change'],
                         [('change','camera',4),('change','camera',2)])

    def test_release_alignment_before_mapping(self):
        calls, states, error = self.run_handoff('camera',{'camera':1,'camera_align':3})
        self.assertIsNone(error)
        self.assertEqual(states['camera_align'],1)

    def test_hung_target_changes_nothing(self):
        calls, _, error = self.run_handoff(states={'camera':3,'camera_align':None})
        self.assertIsNotNone(error)
        self.assertFalse(any(x[0]=='change' for x in calls))

    def test_transitioning_node_changes_nothing(self):
        calls, _, error = self.run_handoff(states={'camera':3,'camera_align':10})
        self.assertIsNotNone(error)
        self.assertFalse(any(x[0]=='change' for x in calls))

    def test_rejected_release_never_prepares_owner(self):
        calls, _, error = self.run_handoff(reject=True)
        self.assertIsNotNone(error)
        self.assertFalse(any(x[0]=='event' for x in calls))

    def test_idle_other_is_not_activated(self):
        calls, _, error = self.run_handoff(states={'camera':1,'camera_align':1})
        self.assertIsNone(error)
        self.assertFalse(any(x[0]=='change' for x in calls))

if __name__ == '__main__': unittest.main()
