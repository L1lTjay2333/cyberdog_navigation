import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rgbd_correspondence_fit import fit,project_points


class CorrespondenceTests(unittest.TestCase):
    def data(self):
        rng=np.random.default_rng(123);xyz=rng.uniform([-.3,-.2,.8],[.3,.2,1.8],(36,3))
        t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',[.4,-.7,.2],degrees=True).as_matrix()
        t[:3,3]=[.006,-.01,.004]
        k=[300,301,320,240];d=[.01,-.02,.001,-.001,0]
        uv=project_points(xyz,t,k,d)
        return dict(depth_xyz_m=xyz.tolist(),color_uv=uv.tolist(),groups=['a']*12+['b']*12+['c']*12,
                    validation_groups=['c'],measurement_evidence='synthetic test only',
                    color_intrinsics=k,distortion=d,initial_depth_to_color=np.eye(4).tolist())

    def test_known_extrinsic_and_independent_validation(self):
        report=fit(self.data())
        self.assertTrue(report['withheld_screen_pass'])
        self.assertLess(report['candidate']['validation']['p95_px'],.001)
        self.assertFalse(report['spatial_registration_verified'])

    def test_heldout_bad_correspondences_are_not_hidden_by_fit(self):
        data=self.data();uv=np.array(data['color_uv']);uv[24:]+=20;data['color_uv']=uv.tolist()
        report=fit(data)
        self.assertLess(report['candidate']['fit']['p95_px'],.001)
        self.assertFalse(report['withheld_screen_pass'])

    def test_planar_or_same_group_evaluation_rejected(self):
        data=self.data();data['validation_groups']=[]
        with self.assertRaises(ValueError):fit(data)
        data=self.data();xyz=np.array(data['depth_xyz_m']);xyz[:,2]=1;data['depth_xyz_m']=xyz.tolist()
        with self.assertRaises(ValueError):fit(data)
