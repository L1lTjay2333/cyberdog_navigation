import base64
import struct
import unittest
import zlib
import numpy as np
try:
    import open3d
except ImportError:
    open3d=None
from test_reconstruction_core import frame


def plane(stamp=1.,depth_mm=1000,translation=None):
    record=frame(stamp,width=80,height=60,step=160,intrinsics=[80.,80.,40.,30.],translation=translation or [0.,0.,0.])
    record['depth_zlib_b64']=base64.b64encode(zlib.compress(struct.pack('<H',depth_mm)*4800)).decode()
    return record


@unittest.skipIf(open3d is None,'Requires isolated Open3D environment')
class TSDFTests(unittest.TestCase):
    def test_repeated_identical_depth_does_not_duplicate_surface_faces(self):
        from reconstruction_tsdf import fuse_depth_records
        once,_,_=fuse_depth_records([plane()],max_blocks=256)
        repeated,_,_=fuse_depth_records([plane(float(i)) for i in (1,2,3)],max_blocks=256)
        self.assertEqual(len(once.triangles),len(repeated.triangles))

    def test_empty_observations_do_not_invent_a_surface(self):
        from reconstruction_tsdf import fuse_depth_records
        mesh,confirmed,report=fuse_depth_records([plane(depth_mm=0)],max_blocks=256)
        self.assertEqual(len(mesh.triangles),0)
        self.assertEqual(len(confirmed.triangles),0)
        self.assertEqual(report['active_blocks'],0)

    def test_single_observation_surface_is_preserved_separately(self):
        from reconstruction_tsdf import fuse_depth_records
        all_mesh,confirmed,report=fuse_depth_records([plane()],max_blocks=256)
        self.assertGreater(len(all_mesh.triangles),100)
        self.assertEqual(len(confirmed.triangles),0)
        self.assertEqual(report['frames'],1)

    def test_repeated_plane_has_confirmed_surface_at_measured_depth(self):
        from reconstruction_tsdf import fuse_depth_records
        all_mesh,confirmed,_=fuse_depth_records([plane(float(i)) for i in (1,2,3)],max_blocks=256)
        self.assertGreater(len(confirmed.triangles),100)
        np.testing.assert_allclose(np.asarray(all_mesh.vertices)[:,2],1.,atol=.025)

    def test_camera_motion_uses_world_to_camera_extrinsic(self):
        from reconstruction_tsdf import fuse_depth_records
        mesh,_,_=fuse_depth_records([plane(),plane(2.,900,[0.,0.,.1])],max_blocks=256)
        np.testing.assert_allclose(np.asarray(mesh.vertices)[:,2],1.,atol=.025)

    def test_coordinate_frame_change_is_rejected(self):
        from reconstruction_tsdf import fuse_depth_records
        with self.assertRaises(ValueError):fuse_depth_records([plane(),dict(plane(2.),frame='map')],max_blocks=256)

    def test_resource_limit_does_not_silently_drop_blocks(self):
        from reconstruction_tsdf import fuse_depth_records
        with self.assertRaises(ValueError):fuse_depth_records([plane()],max_blocks=1)

    def test_range_limit_excludes_far_surface_without_changing_source(self):
        from reconstruction_tsdf import fuse_depth_records
        import copy
        records=[plane(),plane(2.,2500)]
        original=copy.deepcopy(records)
        mesh,_,report=fuse_depth_records(records,max_blocks=256,max_depth=2.)
        self.assertGreater(len(mesh.triangles),100)
        np.testing.assert_allclose(np.asarray(mesh.vertices)[:,2],1.,atol=.025)
        self.assertEqual(report['outside_depth_limit_pixels'],4800)
        self.assertEqual(records,original)

    def test_all_far_frames_return_empty_not_clamped_surface(self):
        from reconstruction_tsdf import fuse_depth_records
        mesh,_,report=fuse_depth_records([plane(depth_mm=2500)],max_blocks=256,max_depth=2.)
        self.assertEqual(len(mesh.triangles),0)
        self.assertEqual(report['active_blocks'],0)

    def test_invalid_depth_limit_is_rejected(self):
        from reconstruction_tsdf import fuse_depth_records
        for value in (float('nan'),float('inf'),0.,4.1):
            with self.subTest(value=value),self.assertRaises(ValueError):
                fuse_depth_records([plane()],max_depth=value)


if __name__=='__main__':unittest.main()
