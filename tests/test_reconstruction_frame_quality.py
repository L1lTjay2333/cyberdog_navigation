import unittest

import numpy as np

from reconstruction_frame_quality import sample_clean_depth


class FrameQualityTests(unittest.TestCase):
    def test_discards_spike_without_filling_hole_or_smearing_depth_edge(self):
        depth = np.full((9, 11), 1000, dtype=np.uint16)
        depth[:, 6:] = 1500
        depth[2, 2] = 820
        depth[5, 2] = 0
        depth[3, 3] = 1005
        clean = sample_clean_depth(depth, 1, .001)
        self.assertTrue(np.isnan(clean[2, 2]))
        self.assertTrue(np.isnan(clean[5, 2]))
        self.assertAlmostEqual(clean[3, 3], 1.001)
        np.testing.assert_allclose(clean[:, 5], 1)
        np.testing.assert_allclose(clean[:, 6], 1.5)

    def test_retains_single_pixel_wide_connected_structure(self):
        depth = np.full((7, 8), 1500, dtype=np.uint16)
        depth[1:6, 3] = 900
        clean = sample_clean_depth(depth, 1, .001)
        np.testing.assert_allclose(clean[1:6, 3], .9)
        np.testing.assert_allclose(sample_clean_depth(depth, 3, .001), clean[::3, ::3])
