import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruction_multiscale import propose,split_tracks,track_step
from reconstruction_registration import apply_transform
from test_reconstruction_refinement import corner


class MultiscaleTests(unittest.TestCase):
    def test_recovers_known_small_error(self):
        target=corner();delta=np.eye(4)
        delta[:3,:3]=Rotation.from_euler('xyz',[1.,-.6,.8],degrees=True).as_matrix()
        delta[:3,3]=[.012,-.008,.009]
        source=apply_transform(target,delta);result=propose(source,target)
        self.assertTrue(result['qualified'],result)
        self.assertEqual(result['decision'],'apply_correction')
        self.assertLess(np.linalg.norm(apply_transform(source,np.array(result['transform']))-target,axis=1).mean(),.002)

    def test_plane_does_not_authorize_unobservable_pose(self):
        plane=corner()[:1600];result=propose(plane+[0,0,.005],plane)
        self.assertFalse(result['qualified'],result)
        np.testing.assert_array_equal(result['transform'],np.eye(4))

    def test_aligned_input_keeps_original_pose(self):
        result=propose(corner(),corner())
        self.assertTrue(result['qualified'],result)
        self.assertEqual(result['decision'],'keep_odometry')
        np.testing.assert_array_equal(result['transform'],np.eye(4))

    def test_large_local_update_is_not_applied_even_if_fit_improves(self):
        result=propose(corner()+[.07,0,0],corner())
        self.assertFalse(result['qualified'],result)
        np.testing.assert_array_equal(result['transform'],np.eye(4))

    def test_too_few_points_does_not_crash_or_correct(self):
        result=propose([[0,0,0]],corner())
        self.assertFalse(result['qualified']);np.testing.assert_array_equal(result['transform'],np.eye(4))

    def test_split_preserves_every_frame_once_and_reanchors_after_failure(self):
        answers=iter([{'qualified':True,'transform':np.eye(4).tolist()},
                      {'qualified':False,'reason':'lost_validation_overlap'},
                      {'qualified':True,'transform':np.eye(4).tolist()}])
        groups=split_tracks([corner()]*4,solver=lambda *args:next(answers))
        self.assertEqual([g['indices'] for g in groups],[[0,1],[2,3]])
        self.assertEqual([i for g in groups for i in g['indices']],list(range(4)))
        np.testing.assert_array_equal(groups[1]['corrections'][0],np.eye(4))

    def test_individually_small_updates_cannot_exceed_cumulative_limit(self):
        prior=np.eye(4);prior[0,3]=.04;delta=np.eye(4);delta[0,3]=.02
        quality,transform=track_step(corner(),prior,[corner()],
            solver=lambda *args:{'qualified':True,'transform':delta.tolist()})
        self.assertIsNone(transform);self.assertEqual(quality['reason'],'cumulative_correction_limit')

    def test_pose_correction_matches_world_cloud_correction(self):
        from reconstruction_segments import corrected_record
        from reconstruction_core import decode_depth_frame
        from test_reconstruction_core import frame
        original=frame();delta=np.eye(4)
        delta[:3,:3]=Rotation.from_euler('z',2,degrees=True).as_matrix();delta[:3,3]=[.01,-.02,.005]
        updated=corrected_record(original,delta)
        np.testing.assert_allclose(decode_depth_frame(updated),apply_transform(decode_depth_frame(original),delta),atol=1e-12)
        self.assertEqual(updated['depth_zlib_b64'],original['depth_zlib_b64'])
        self.assertEqual(updated['stamp'],original['stamp'])
        self.assertNotEqual(updated['translation'],original['translation'])
