import unittest
from sensor_consistency import sensor_consistency

class SensorConsistencyTests(unittest.TestCase):
    def test_fresh_contradictory_readings_are_reported_not_replaced(self):
        s={'emitted_unix':100,'local_received_unix':110,
           'laser_overlay':{'received_unix':100,'central_range_m':.21},
           'cloud':{'front_check':{'stamp':100,'distance_m':2.69}}}
        r=sensor_consistency(s,110)
        self.assertEqual(r['status'],'conflict');self.assertEqual(r['laser_m'],.21)
        self.assertEqual(sensor_consistency(s,112)['status'],'unknown')
        s['cloud']['front_check']['distance_m']=.4
        self.assertEqual(sensor_consistency(s,110)['status'],'available')

    def test_missing_camera_is_not_assumed_clear(self):
        self.assertEqual(sensor_consistency({},110)['status'],'unknown')
