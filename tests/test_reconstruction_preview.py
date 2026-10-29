import tempfile
from pathlib import Path
import unittest
import numpy as np
import open3d as o3d
from reconstruction_offline_worker import preview_mesh


class PreviewTests(unittest.TestCase):
    def test_large_closed_surface_is_simplified_without_dropping_random_faces(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);path=root/'sphere.ply'
            mesh=o3d.geometry.TriangleMesh.create_sphere(resolution=200)
            o3d.io.write_triangle_mesh(str(path),mesh);before=path.read_bytes()
            descriptor=preview_mesh(path,root,'preview')
            self.assertEqual(descriptor['preview_method'],'quadric_decimation')
            self.assertGreater(descriptor['full_triangles'],150000)
            self.assertLessEqual(descriptor['displayed_triangles'],150000)
            values=np.fromfile(root/'preview.bin',dtype='<f4').reshape(-1,3,3)
            vertices,inverse=np.unique(values.reshape(-1,3),axis=0,return_inverse=True)
            rebuilt=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),o3d.utility.Vector3iVector(inverse.reshape(-1,3)))
            self.assertTrue(rebuilt.is_edge_manifold(allow_boundary_edges=False))
            self.assertEqual(before,path.read_bytes())
