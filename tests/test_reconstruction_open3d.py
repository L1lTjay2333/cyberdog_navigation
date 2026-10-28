import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
try:
    import open3d
except ImportError:
    open3d = None
if open3d is not None:
    from benchmark_open3d_registration import assess_candidate, compare_pair, benchmark
from test_reconstruction_refinement import corner
from test_reconstruction_core import frame


@unittest.skipIf(open3d is None, 'Run in the isolated D-drive Open3D environment')
class Open3DChecks(unittest.TestCase):
    def test_complete_archive_preferred_without_claiming_continuous_tracking(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'frames.jsonl').write_text(json.dumps(frame())+'\n'+json.dumps(frame(2.))+'\n', encoding='utf-8')
            (root/'frame_01.json').write_text(json.dumps(frame()), encoding='utf-8')
            metadata = {'accepted_frames':2,'raw_archive':{'complete':True,'frames':2},'error':'capacity stop'}
            (root/'summary.json').write_text(json.dumps(metadata), encoding='utf-8')
            with patch('benchmark_open3d_registration.compare_pair', return_value={'test_pair':True}):
                result = benchmark(root)
            self.assertEqual(result['input_frames'], 2)
            self.assertEqual(result['input_source'], 'frames.jsonl')
            self.assertTrue(result['all_accepted_frames_evaluated'])
            self.assertFalse(result['full_session_replay'])
            self.assertEqual(result['capture_termination'], 'capacity stop')
            self.assertEqual(len(result['pairs']), 1)

    def test_archive_coordinate_frame_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'frames.jsonl').write_text(json.dumps(frame())+'\n'+json.dumps(frame(2., frame='map'))+'\n', encoding='utf-8')
            with self.assertRaises(ValueError):benchmark(root)

    def test_scaled_matrix_cannot_pass_as_rigid_motion(self):
        transform = np.eye(4);transform[0, 0] = 2.
        result = assess_candidate(corner(), corner(), transform)
        self.assertFalse(result['qualified'])
        self.assertEqual(result['reason'], 'invalid_rigid_transform')

    def test_single_plane_not_certified_even_with_zero_residual(self):
        plane = corner()[:1600]
        result = assess_candidate(plane, plane, np.eye(4))
        self.assertFalse(result['qualified'])
        self.assertEqual(result['reason'], 'degenerate_geometry')

    def test_no_overlap_not_certified(self):
        result = assess_candidate(corner()+[2., 0., 0.], corner(), np.eye(4))
        self.assertFalse(result['qualified'])
        self.assertEqual(result['reason'], 'insufficient_overlap')

    def test_known_correction_is_checked_on_same_held_out_points(self):
        delta = np.array([.008, -.006, .005])
        transform = np.eye(4);transform[:3, 3] = -delta
        result = assess_candidate(corner()+delta, corner(), transform)
        self.assertTrue(result['qualified'], result)
        self.assertLess(result['after_rmse_m'], 1e-10)
        self.assertGreater(result['before_rmse_m'], .001)
        self.assertEqual(result['retained_validation_overlap'], 1.)

    def test_open3d_solver_corrects_known_local_error(self):
        result = compare_pair(corner()+[.008, -.006, .005], corner())
        self.assertTrue(result['open3d']['common_check']['qualified'], result)
        self.assertLess(result['open3d']['common_check']['after_rmse_m'], .002)


if __name__ == '__main__':unittest.main()
