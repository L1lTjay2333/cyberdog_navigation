import unittest
from unittest.mock import patch
import numpy as np
from reconstruction_local import joint_align,mls_project,validate_joint_step


class LocalTests(unittest.TestCase):
    def test_large_joint_update_is_bounded_after_backtracking(self):
        cloud=np.zeros((100,3));delta=np.eye(4);delta[0,3]=.08
        with patch('reconstruction_local.assess_candidate',return_value={'qualified':True}) as assess:
            poses,trials=validate_joint_step([cloud,cloud],[(1,0,delta)],[np.eye(4),delta])
        self.assertEqual(trials[-1]['scale'],.5)
        self.assertLessEqual(poses[1][0,3],.05);self.assertEqual(assess.call_count,1)

    def test_normal_gate_preserves_two_close_opposite_surfaces(self):
        x,y=np.meshgrid(np.linspace(-.03,.03,20),np.linspace(-.03,.03,20))
        a=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        p=np.concatenate([a,a+[0,0,.004]])
        normals=np.concatenate([np.tile([0,0,1],(len(a),1)),np.tile([0,0,-1],(len(a),1))])
        out,report=mls_project(p,normals=normals)
        np.testing.assert_allclose(out,p,atol=1e-12)
        self.assertEqual(report['normal_gate_deg'],30)

    def test_malformed_normals_are_rejected(self):
        with self.assertRaises(ValueError):mls_project([[0,0,0]],normals=[[0,0,np.nan]])

    def test_joint_step_backtracks_without_applying_rejected_update(self):
        cloud=np.zeros((100,3));delta=np.eye(4);delta[0,3]=.02
        with patch('reconstruction_local.assess_candidate',side_effect=[{'qualified':False},{'qualified':True}]):
            poses,trials=validate_joint_step([cloud,cloud],[(1,0,delta)],[np.eye(4),delta])
        self.assertEqual(trials[-1]['scale'],.5)
        self.assertAlmostEqual(poses[1][0,3],.01)
        self.assertEqual(delta[0,3],.02)

    def test_failed_backtracking_returns_no_pose(self):
        cloud=np.zeros((100,3))
        with patch('reconstruction_local.assess_candidate',return_value={'qualified':False}):
            poses,trials=validate_joint_step([cloud,cloud],[(1,0,np.eye(4))],[np.eye(4)]*2)
        self.assertIsNone(poses);self.assertEqual(len(trials),4)

    def test_disconnected_graph_never_returns_corrections(self):
        with patch('reconstruction_local.propose',return_value={'qualified':False,'reason':'insufficient_overlap'}):
            poses,report=joint_align([np.zeros((100,3))]*3)
        self.assertIsNone(poses);self.assertFalse(report['accepted'])

    def test_mls_reduces_noisy_plane_without_adding_points(self):
        rng=np.random.default_rng(10)
        x,y=np.meshgrid(np.linspace(-.1,.1,40),np.linspace(-.1,.1,40))
        points=np.column_stack((x.ravel(),y.ravel(),rng.normal(0,.001,x.size)))
        before=points.copy();result,report=mls_project(points)
        self.assertLess(np.std(result[:,2]),np.std(points[:,2])*.7)
        self.assertEqual(result.shape,points.shape)
        self.assertLessEqual(np.linalg.norm(result-points,axis=1).max(),.00500001)
        np.testing.assert_array_equal(points,before)

    def test_mls_keeps_thin_line(self):
        points=np.column_stack((np.linspace(0,.1,100),np.zeros((100,2))))
        result,_=mls_project(points)
        np.testing.assert_array_equal(result,points)

    def test_invalid_input_rejected(self):
        with self.assertRaises(ValueError):mls_project([[np.nan,0,0]])
        with self.assertRaises(ValueError):mls_project([[0,0,0]],max_shift=.1)
