import base64
import struct
import zlib
import unittest
import numpy as np
try:
    import open3d
except ImportError:
    open3d=None
from test_reconstruction_core import frame


@unittest.skipIf(open3d is None,'Requires isolated Open3D runtime')
class MeshTests(unittest.TestCase):
    def test_depth_is_metric_and_extrinsic_is_world_to_camera(self):
        from reconstruction_mesh import depth_and_camera
        f=frame(translation=[1.,2.,3.])
        depth,intrinsic,extrinsic=depth_and_camera(f)
        np.testing.assert_allclose(depth,1.)
        np.testing.assert_allclose(extrinsic[:3,3],[-1.,-2.,-3.])
        self.assertEqual(intrinsic.width,2)

    def test_full_quaternion_is_inverted_without_scale_change(self):
        from reconstruction_mesh import depth_and_camera
        f=frame(translation=[1.,0.,0.],quaternion=[0.,float(np.sin(np.pi/4)),0.,float(np.cos(np.pi/4))])
        _,_,matrix=depth_and_camera(f)
        np.testing.assert_allclose(matrix.dot([2.,0.,0.,1.])[:3],[0.,0.,1.],atol=1e-10)

    def test_invalid_compressed_input_is_rejected(self):
        from reconstruction_mesh import depth_and_camera
        with self.assertRaises(ValueError):depth_and_camera(frame(depth_zlib_b64='invalid'))

    def test_synthetic_plane_generates_surface_at_known_world_depth(self):
        from reconstruction_mesh import integrate_records
        f=frame(width=80,height=60,step=160,intrinsics=[80.,80.,40.,30.])
        f['depth_zlib_b64']=base64.b64encode(zlib.compress(struct.pack('<H',1000)*4800)).decode()
        mesh,report=integrate_records([f,dict(f,stamp=2.)])
        self.assertGreater(len(mesh.triangles),100)
        np.testing.assert_allclose(np.asarray(mesh.vertices)[:,2],1.,atol=.025)
        self.assertEqual(report['integrated_frames'],2)

    def test_coordinate_change_rejected_before_fusion(self):
        from reconstruction_mesh import integrate_records
        with self.assertRaises(ValueError):integrate_records([frame(),frame(2.,frame='map')])


if __name__=='__main__':unittest.main()
