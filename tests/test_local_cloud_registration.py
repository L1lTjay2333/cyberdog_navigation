import unittest
import numpy as np
from local_cloud_registration import estimate, LocalRegistration


class LocalCloudRegistrationTest(unittest.TestCase):
    def setUp(self):
        random = np.random.default_rng(11)
        yz = random.uniform(-1, 1, (2500, 2))
        self.wall = np.column_stack((np.zeros(len(yz)), yz))

    def test_wall_does_not_invent_along_wall_correction(self):
        _, result = estimate(self.wall + [.012, .022, 0], self.wall)
        self.assertLess(abs(result['dx'] + .012), .002)
        self.assertLess(abs(result['dy']), 1e-8)

    def test_corner_and_no_overlap(self):
        corner = np.vstack((self.wall, self.wall[:, [1, 0, 2]]))
        _, result = estimate(corner + [.012, .022, 0], corner)
        self.assertLess(result['after'], result['before'] * .5)
        self.assertIsNone(estimate(corner + 50, corner))

    def test_frame_change_and_gap_reset(self):
        registration = LocalRegistration()
        for stamp, frame in [(10, 'map'), (15, 'map'), (15.2, 'laser_odom')]:
            cloud = dict(aligned=True, frame=frame, stamp=stamp, points_xyz=self.wall.tolist())
            registration.apply(cloud, 'room')
            self.assertFalse(cloud['registration']['applied'])
