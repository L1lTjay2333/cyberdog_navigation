import math
import unittest
import numpy as np
from test_reconstruction_core import frame
import reconstruction_core as core


class MovingTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(core,'MovingModel'),'Moving capture model is not implemented')

    def test_slow_motion_is_allowed_but_static_mode_stays_strict(self):
        first,second=frame(),frame(1.5,translation=[.1,0.,0.])
        moving=core.MovingModel();moving.add(first,stride=1)
        self.assertTrue(moving.add(second,stride=1))
        self.assertAlmostEqual(moving.stats()['travel_m'],.1)
        static=core.StaticModel();static.add(first,stride=1)
        with self.assertRaises(ValueError):static.add(second,stride=1)

    def test_forward_camera_motion_reconstructs_same_plane(self):
        moving=core.MovingModel()
        moving.add(frame(),stride=1)
        moving.add(frame(1.5,values=(900,900,900,900),translation=[0.,0.,.1]),stride=1)
        points=moving.points()
        self.assertGreater(len(points),0)
        np.testing.assert_allclose([p[2] for p in points],1.)

    def test_jump_fast_rotation_and_time_gap_rejected_before_mutation(self):
        for bad in [frame(1.5,translation=[.5,0.,0.]),frame(1.01,translation=[.1,0.,0.]),
                    frame(1.2,quaternion=[0.,0.,math.sin(.4),math.cos(.4)]),
                    frame(5.,translation=[.1,0.,0.]),frame(1.5,frame='map')]:
            with self.subTest(bad=bad['stamp']):
                model=core.MovingModel();model.add(frame(),stride=1)
                before=model.stats().copy()
                with self.assertRaises(ValueError):model.add(bad,stride=1)
                self.assertEqual(before,model.stats())

    def test_duplicate_pose_is_not_an_extra_trajectory_sample(self):
        model=core.MovingModel();model.add(frame(),stride=1)
        self.assertFalse(model.add(frame(),stride=1))
        self.assertEqual(len(model.trajectory),1)

    def test_mode_reports_no_slam_or_calibration_claim(self):
        model=core.MovingModel();model.add(frame(),stride=1)
        stats=model.stats()
        self.assertEqual(stats['mode'],'moving_local_point_cloud')
        self.assertFalse(stats['loop_closure'])
        self.assertFalse(stats['scale_calibrated'])
        self.assertEqual(len(model.last_pose),2)

    def test_latest_pose_tracks_accepted_motion(self):
        model=core.MovingModel();model.add(frame(),stride=1)
        model.add(frame(1.5,translation=[.1,0.,0.]),stride=1)
        np.testing.assert_allclose(model.last_pose[0],[.1,0,0])
        np.testing.assert_allclose(model.anchor[0],[0,0,0])

    def test_capture_distance_is_bounded(self):
        model=core.MovingModel()
        model.add(frame(),stride=1)
        for i in range(1,20):model.add(frame(1+i*.5,translation=[i*.1,0.,0.]),stride=1)
        with self.assertRaises(ValueError):model.add(frame(11.,translation=[2.01,0.,0.]),stride=1)


if __name__=='__main__':unittest.main()
