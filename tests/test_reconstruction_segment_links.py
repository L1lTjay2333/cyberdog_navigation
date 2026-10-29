import unittest
import numpy as np
from reconstruction_segment_links import check_link


class LinkTests(unittest.TestCase):
    def test_one_direction_failure_cannot_create_link(self):
        answers=iter([{'qualified':True,'transform':np.eye(4).tolist()},{'qualified':False}])
        result=check_link(np.zeros((100,3)),np.zeros((100,3)),lambda *args:next(answers))
        self.assertFalse(result['accepted'])

    def test_inverse_pair_is_cycle_consistent(self):
        forward=np.eye(4);forward[0,3]=.02
        answers=iter([{'qualified':True,'transform':forward.tolist()},
                      {'qualified':True,'transform':np.linalg.inv(forward).tolist()}])
        result=check_link(np.zeros((100,3)),np.zeros((100,3)),lambda *args:next(answers))
        self.assertTrue(result['accepted']);self.assertAlmostEqual(result['cycle_shift_m'],0.)

    def test_two_qualified_but_inconsistent_directions_are_rejected(self):
        delta=np.eye(4);delta[0,3]=.02
        result=check_link(np.zeros((100,3)),np.zeros((100,3)),
                          lambda *args:{'qualified':True,'transform':delta.tolist()})
        self.assertFalse(result['accepted']);self.assertEqual(result['reason'],'cycle_inconsistent')
