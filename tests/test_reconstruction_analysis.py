import unittest
from test_reconstruction_core import frame
from reconstruction_analysis import compare_frames,replay_voxel_counts


class AnalysisTests(unittest.TestCase):
    def test_identical_frame_has_zero_residual(self):
        report=compare_frames(frame(),frame(2.))
        self.assertEqual(report['absolute_depth_residual_m']['max'],0.)
        self.assertEqual(report['comparable_fraction'],1.)

    def test_pose_compensation_of_camera_moving_towards_plane(self):
        report=compare_frames(frame(),frame(2.,values=(900,900,900,900),translation=[0.,0.,.1]))
        self.assertAlmostEqual(report['absolute_depth_residual_m']['max'],0.)

    def test_uncorrected_pose_error_is_not_hidden(self):
        report=compare_frames(frame(),frame(2.,values=(900,900,900,900)))
        self.assertAlmostEqual(report['absolute_depth_residual_m']['median'],.1)
        self.assertEqual(report['within_2cm_fraction'],0.)

    def test_voxel_support_counts_frames_not_samples(self):
        rows=replay_voxel_counts([frame(),frame(2.)],sizes=(.01,.02))
        self.assertTrue(all(r['supported_by_2_frames']==r['voxels'] for r in rows))

    def test_no_comparable_pixels_are_not_reported_as_zero_error(self):
        r=compare_frames(frame(),frame(2.,translation=[3.,0.,0.]))
        self.assertEqual(r['absolute_depth_residual_m'],{'count':0})
        self.assertIsNone(r['within_2cm_fraction'])


if __name__=='__main__':unittest.main()
