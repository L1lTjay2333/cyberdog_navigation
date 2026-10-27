import math
import unittest
from depth_scan_geometry import points_to_scan

class DepthScanTests(unittest.TestCase):
    def test_measured_nearest_return_kept_and_unseen_rays_unknown(self):
        result=points_to_scan([(2.5,0,0),(3.,0,.2),(.3,0,.1)])
        self.assertAlmostEqual(result[180],.3)
        self.assertTrue(math.isnan(result[0]))
        self.assertEqual(sum(math.isfinite(v) for v in result),1)

    def test_floor_ceiling_missing_and_behind_points_do_not_make_free_space(self):
        result=points_to_scan([(1,0,-.3),(1,0,.8),(-1,0,0),(float('nan'),0,0),(8,0,0)])
        self.assertTrue(all(math.isnan(v) for v in result))

    def test_left_right_angles_are_not_reversed(self):
        result=points_to_scan([(2,2,0),(2,-2,0)])
        self.assertAlmostEqual(result[270],math.sqrt(8));self.assertAlmostEqual(result[90],math.sqrt(8))
