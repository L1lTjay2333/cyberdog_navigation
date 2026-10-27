import math
from types import SimpleNamespace
import unittest
from obstacle_distance_probe import scan_points, summarize
from cloud_geometry import transform_xyz

class DistanceTests(unittest.TestCase):
    def test_angles_and_invalid_ranges(self):
        scan=SimpleNamespace(angle_min=-math.pi/2,angle_increment=math.pi/2,
            range_min=.1,range_max=10.,ranges=[1.,2.,float('nan'),float('inf'),0.,11.])
        pts=scan_points(scan)
        self.assertEqual(len(pts),2)
        self.assertAlmostEqual(pts[0][1],-1.)
        self.assertAlmostEqual(pts[1][0],2.)
    def test_reject_bad_scan(self):
        with self.assertRaises(ValueError):
            scan_points(SimpleNamespace(angle_min=0.,angle_increment=0.,range_min=.1,range_max=10.,ranges=[2.]))
    def test_slant_range_differs_from_forward(self):
        p=[2*math.cos(math.pi/3),2*math.sin(math.pi/3),0.]
        r=summarize([p],[.1,5,-3,3,-1,1],1)
        self.assertAlmostEqual(r['forward_x_min_m'],1.)
        self.assertAlmostEqual(r['horizontal_range_min_m'],2.)
    def test_roi_zero_nan_and_missing(self):
        r=summarize([[0,0,0],[float('nan'),0,0],[1,2,0]],[.1,5,-.3,.3,-1,1],1)
        self.assertFalse(r['valid'])
        self.assertNotIn('forward_x_min_m',r)
    def test_outlier_is_visible(self):
        pts=[[.2,0,0]]+[[2.,0,0] for _ in range(20)]
        r=summarize(pts,[.1,5,-.3,.3,-1,1],3)
        self.assertEqual(r['forward_x_min_m'],.2)
        self.assertEqual(r['forward_x_median_m'],2.)
        self.assertEqual(r['forward_x_p10_m'],2.)
    def test_optical_transform_and_origin_offset(self):
        pts=transform_xyz([[0,0,2]],[.15,0,0],[.5,-.5,.5,-.5])
        self.assertAlmostEqual(pts[0][0],2.15)
        self.assertAlmostEqual(pts[0][1],0.)
        self.assertAlmostEqual(pts[0][2],0.)

if __name__=='__main__': unittest.main()
