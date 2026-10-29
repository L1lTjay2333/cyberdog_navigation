import unittest
import numpy as np
from test_reconstruction_core import frame
from reconstruction_noise_audit import depth_layers,select_windows
from reconstruction_postprocess import filter_depth


class NoiseAuditTests(unittest.TestCase):
    def test_layers_partition_valid_depth_without_inventing_pixels(self):
        depth=np.ones((20,20));depth[:,10:]=2;depth[4,4]=0
        interior,edges,_=depth_layers(depth);filtered,_=filter_depth(depth)
        np.testing.assert_array_equal(interior|edges,filtered>0)
        self.assertFalse((interior&edges).any());self.assertTrue(edges[10,9]);self.assertTrue(interior[10,3])

    def test_no_static_moving_pair_is_not_faked(self):
        with self.assertRaises(ValueError):select_windows([frame(float(i+1)) for i in range(20)])

    def test_windows_have_equal_count_and_explicit_motion_thresholds(self):
        frames=[frame(float(i+1),translation=[max(0,i-12)*.03,0,0]) for i in range(30)]
        choices=select_windows(frames)
        self.assertEqual(choices['static']['end']-choices['static']['start'],10)
        self.assertEqual(choices['moving']['end']-choices['moving']['start'],10)
        self.assertGreaterEqual(choices['moving']['start'],choices['static']['end'])
        self.assertGreaterEqual(choices['moving']['max_translation_m'],.15)
