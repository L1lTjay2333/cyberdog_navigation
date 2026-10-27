import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruction_registration import register_clouds, apply_transform
from reconstruction_submaps import DiskSubmaps
from reconstruction_replay import replay
from test_reconstruction_core import frame
from reconstruction_core import decode_depth_frame
from reconstruction_archive import FrameArchive


def corner():
    grid = np.linspace(-.5, .5, 40)
    a, b = np.meshgrid(grid, grid)
    a, b = a.ravel(), b.ravel()
    # Three perpendicular planes constrain all six rigid-body degrees of freedom.
    return np.concatenate([np.column_stack((a, b, np.full_like(a, .7))),
                           np.column_stack((a, np.full_like(a, .7), b)),
                           np.column_stack((np.full_like(a, .7), a, b))])


class RegistrationTests(unittest.TestCase):
    def test_recovers_small_residual_pose_error_with_outliers(self):
        target = corner()
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_euler('xyz', [1., -.6, .8], degrees=True).as_matrix()
        delta[:3, 3] = [.012, -.008, .009]
        source = apply_transform(target, delta)
        source = np.concatenate([source, np.random.default_rng(42).uniform(3., 4., (200, 3))])
        result = register_clouds(source, target)
        self.assertTrue(result['accepted'], result)
        corrected = apply_transform(source[:len(target)], np.array(result['transform']))
        self.assertLess(np.linalg.norm(corrected-target, axis=1).mean(), .002)
        self.assertLess(result['candidate_validation_rmse_m'], result['before_validation_rmse_m'])

    def test_single_plane_is_rejected_as_ambiguous(self):
        plane = corner()[:1600]
        result = register_clouds(plane+[0., 0., .005], plane)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'degenerate_geometry')
        np.testing.assert_array_equal(result['transform'], np.eye(4))

    def test_no_overlap_is_not_fused(self):
        result = register_clouds(corner()+[2., 0., 0.], corner())
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'insufficient_overlap')

    def test_world_axes_rotated_90_degrees_still_recover_local_error(self):
        # Tests coordinate-frame invariance, not a real camera 90-degree turn.
        target = corner().dot(Rotation.from_euler('z', 90, degrees=True).as_matrix().T)
        result = register_clouds(target+[.008, -.006, .005], target)
        self.assertTrue(result['accepted'], result)
        corrected = apply_transform(target+[.008, -.006, .005], np.array(result['transform']))
        self.assertLess(np.linalg.norm(corrected-target, axis=1).mean(), .002)

    def test_depth_projection_uses_full_90_degree_camera_orientation(self):
        # Verifies the optical-to-world rotation; does not assert tracking across
        # two camera views without overlap.
        original = decode_depth_frame(frame(), stride=1)
        q = Rotation.from_euler('y', 90, degrees=True).as_quat().tolist()
        rotated = decode_depth_frame(frame(quaternion=q), stride=1)
        expected = original.dot(Rotation.from_quat(q).as_matrix().T)
        np.testing.assert_allclose(rotated, expected, atol=1e-12)

    def test_large_pose_error_cannot_be_accepted_as_local_refinement(self):
        result = register_clouds(corner()+[.15, .15, .15], corner())
        self.assertFalse(result['accepted'], result)
        np.testing.assert_array_equal(result['transform'], np.eye(4))

    def test_bad_input_and_too_few_points(self):
        with self.assertRaises(ValueError):
            register_clouds([[float('nan'), 0., 0.]], corner())
        self.assertFalse(register_clouds([[0., 0., 0.]], corner())['accepted'])

    def test_already_aligned_geometry_does_not_invent_correction(self):
        result = register_clouds(corner(), corner())
        self.assertTrue(result['accepted'], result)
        np.testing.assert_allclose(result['transform'], np.eye(4), atol=1e-10)


class SubmapTests(unittest.TestCase):
    def test_complete_raw_archive_replays_all_frames_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = FrameArchive(root/'frames.jsonl')
            archive.append(frame());archive.append(frame(2.))
            # Sparse keyframe files coexist, but must not be replayed a second time.
            (root/'frame_01.json').write_text(json.dumps(frame()), encoding='utf-8')
            metadata = {'accepted_frames':2, 'raw_archive':archive.stats(2)}
            (root/'summary.json').write_text(json.dumps(metadata), encoding='utf-8')
            result = replay(root, root/'output')
            self.assertEqual(result['submaps']['frames'], 2)
            self.assertTrue(result['full_session_replay'])

    def test_archive_quota_marks_incomplete_and_never_appends_after_gap(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'frames.jsonl'
            archive = FrameArchive(path, max_bytes=2000)
            archive.append(frame())
            archive.append({'large':'x'*3000})
            archive.append(frame(3.))
            self.assertEqual(archive.frames, 1)
            self.assertFalse(archive.stats(3)['complete'])
            self.assertTrue(archive.stats(3)['truncated'])
            self.assertEqual(len(path.read_text().splitlines()), 1)

    def test_rollover_keeps_old_surfaces_and_overlap_support(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = DiskSubmaps(Path(temporary)/'new', max_voxels=5)
            a = np.array([[.002, 0., 0.], [.022, 0., 0.], [.042, 0., 0.]])
            b = np.array([[.042, 0., 0.], [.062, 0., 0.], [.082, 0., 0.]])
            c = np.array([[.082, 0., 0.], [.102, 0., 0.], [.122, 0., 0.]])
            store.add(a);store.add(b);store.add(c)
            report = store.finish()
            self.assertEqual(report['frames'], 3)
            self.assertEqual(len(report['parts']), 2)
            self.assertTrue(all(part['voxels'] <= 5 for part in report['parts']))
            for part in report['parts']:
                with np.load(store.folder/part['file']) as saved:
                    self.assertEqual(int((saved['support'] >= 2).sum()), 1)
            self.assertTrue((store.folder/'manifest.json').is_file())

    def test_oversized_pair_does_not_discard_active_map(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = DiskSubmaps(Path(temporary)/'new', max_voxels=2)
            store.add([[0., 0., 0.], [.02, 0., 0.]])
            before = set(store.cells)
            with self.assertRaises(ValueError):
                store.add([[1., 0., 0.], [1.02, 0., 0.]])
            self.assertEqual(set(store.cells), before)
            self.assertEqual(store.frames, 1)

    def test_disk_quota_failure_preserves_in_memory_data(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = DiskSubmaps(Path(temporary)/'new', max_bytes=1)
            store.add([[0., 0., 0.]])
            with self.assertRaises(ValueError):store.finish()
            self.assertEqual(len(store.cells), 1)
            self.assertFalse(store.closed)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):DiskSubmaps(temporary)

    def test_replay_rejects_camera_frame_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'frame_01.json').write_text(json.dumps(frame()), encoding='utf-8')
            (root/'frame_02.json').write_text(json.dumps(frame(2., frame='map')), encoding='utf-8')
            with self.assertRaises(ValueError):replay(root, root/'output')

    def test_refinement_loss_stops_without_falling_back_to_odometry(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'frame_01.json').write_text(json.dumps(frame()), encoding='utf-8')
            (root/'frame_02.json').write_text(json.dumps(frame(2.)), encoding='utf-8')
            result = replay(root, root/'output', refine=True)
            self.assertEqual(result['submaps']['frames'], 1)
            self.assertFalse(result['attempts'][-1]['fused'])
            self.assertEqual(result['control_requests'], 0)


if __name__ == '__main__':unittest.main()
