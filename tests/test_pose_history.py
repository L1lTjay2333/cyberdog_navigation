import unittest,math
from pose_history import PoseHistory,CompensatedPose,ScanOdometry

class PoseHistoryTests(unittest.TestCase):
    def test_interpolates_position_and_yaw(self):
        h=PoseHistory();h.add(10,[0,0,0],[0,0,0,1],'laser_odom');h.add(10.1,[.1,0,0],[0,0,math.sin(.05),math.cos(.05)],'laser_odom')
        p=h.at(10.05);self.assertAlmostEqual(p['x'],.05);self.assertAlmostEqual(p['yaw'],.05)
    def test_no_latest_substitution_for_old_depth(self):
        h=PoseHistory();h.add(10,[0,0,0],[0,0,0,1],'laser_odom')
        with self.assertRaises(ValueError):h.at(9.8)
        with self.assertRaises(ValueError):h.latest(10.4)
    def test_rejects_frame_and_bad_quaternion(self):
        h=PoseHistory();h.add(10,[0,0,0],[0,0,0,1],'map');h.add(10,[0,0,0],[0,0,0,0],'laser_odom')
        self.assertEqual(len(h.samples),0)
    def test_long_gap_not_interpolated(self):
        h=PoseHistory();h.add(10,[0,0,0],[0,0,0,1],'laser_odom');h.add(10.3,[0,0,0],[0,0,0,1],'laser_odom')
        with self.assertRaises(ValueError):h.at(10.1)

class CompensationTests(unittest.TestCase):
    def fixture(self):
        slam=PoseHistory();odom=PoseHistory('odom',600)
        slam.add(10,[2,3,0],[0,0,math.sin(math.pi/4),math.cos(math.pi/4)],'laser_odom')
        for i in range(11):odom.add(10+i*.05,[i*.005,0,.24],[0,0,0,1],'odom')
        return CompensatedPose(slam,odom),slam,odom
    def test_measured_increment_rotated_into_map_not_absolute_odom(self):
        estimator,_,_=self.fixture();p=estimator.latest(10.5)
        self.assertAlmostEqual(p['x'],2);self.assertAlmostEqual(p['y'],3.05);self.assertEqual(p['z'],0)
        self.assertAlmostEqual(p['slam_age_s'],.5)
    def test_depth_acquisition_interpolated_not_latest_substitution(self):
        estimator,_,_=self.fixture();p=estimator.at(10.425,10.5)
        self.assertAlmostEqual(p['y'],3.0425)
    def test_anchor_and_current_freshness(self):
        estimator,_,_=self.fixture()
        with self.assertRaises(ValueError):estimator.latest(10.8)
        with self.assertRaises(ValueError):estimator.latest(12)
    def test_missing_anchor_odom_never_identity(self):
        estimator,_,odom=self.fixture();odom.samples.popleft();odom.samples.popleft()
        with self.assertRaises(ValueError):estimator.latest(10.5)
    def test_odom_jump_blocks(self):
        estimator,_,odom=self.fixture();odom.add(10.55,[2,0,.24],[0,0,0,1],'odom')
        with self.assertRaises(ValueError):estimator.latest(10.55)

class ScanOdometryTests(unittest.TestCase):
    def fixture(self):
        odom=PoseHistory('odom',600);odom.add(10,[10,20,.24],[0,0,0,1],'odom')
        anchor=PoseHistory.make(10,[2,3,0],[0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
        return ScanOdometry(anchor,odom),odom
    def test_slam_latency_does_not_interrupt_relative_turn(self):
        scan,odom=self.fixture()
        for i in range(1,81):
            yaw=i*.1;odom.add(10+i*.1,[10,20,.24],[0,0,math.sin(yaw/2),math.cos(yaw/2)],'odom')
            p=scan.latest(10+i*.1)
        self.assertAlmostEqual(p['yaw'],math.atan2(math.sin(math.pi/2+8),math.cos(math.pi/2+8)))
        self.assertEqual(p['source'],'scan_anchor_plus_measured_odom');self.assertEqual(p['z'],0)
    def test_increment_rotated_into_initial_map_frame(self):
        scan,odom=self.fixture();odom.add(10.1,[10.04,20,.24],[0,0,0,1],'odom')
        p=scan.latest(10.1);self.assertAlmostEqual(p['x'],2);self.assertAlmostEqual(p['y'],3.04)
    def test_stale_odom_and_runtime_rejected(self):
        scan,odom=self.fixture()
        with self.assertRaises(ValueError):scan.latest(10.2)
        odom.add(53,[10,20,.24],[0,0,0,1],'odom')
        with self.assertRaises(ValueError):scan.latest(53)
    def test_jump_and_translation_rejected(self):
        scan,odom=self.fixture();odom.add(10.1,[10.09,20,.24],[0,0,0,1],'odom')
        with self.assertRaises(ValueError):scan.latest(10.1)
        scan,odom=self.fixture()
        for i in range(1,5):
            odom.add(10+i*.1,[10+i*.04,20,.24],[0,0,0,1],'odom')
            if i<4:scan.latest(10+i*.1)
        with self.assertRaises(ValueError):scan.latest(10.4)
