import math
import unittest
from types import SimpleNamespace as N
from scan_distance import report_scan

class ScanDistanceTests(unittest.TestCase):
    def report(self,ranges,age=0,stamp=1000000000):
        scan=N(header=N(frame_id='laser_frame',stamp=N(sec=stamp//1000000000,nanosec=stamp%1000000000)),
               angle_min=0.,angle_increment=.001,range_min=.01,range_max=30.,ranges=ranges)
        tf=N(translation=N(x=.179,y=0.,z=.0837),rotation=N(x=0.,y=0.,z=0.,w=1.))
        return report_scan(scan,tf,stamp+int(age*1000000000))
    def test_body_origin_not_sensor_origin(self):
        result=self.report([2.,2.,2.]);self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['raw_range_min_m'],2.)
        self.assertAlmostEqual(result['forward_x_min_m'],2.179,places=4)
    def test_near_outlier_not_hidden(self):
        result=self.report([.2]+[2.]*20)
        self.assertLess(result['forward_x_min_m'],.4)
        self.assertGreater(result['forward_x_median_m'],2.)
    def test_stale_and_zero_stamp_invalid(self):
        self.assertFalse(self.report([2.]*3,age=2.)['valid'])
        self.assertFalse(self.report([2.]*3,stamp=0)['valid'])
    def test_no_data_is_not_clear(self):
        self.assertFalse(self.report([float('nan'),float('inf'),0.])['valid'])

if __name__=='__main__':unittest.main()
