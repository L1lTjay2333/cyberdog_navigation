import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from rgbd_projection_check import foreground_edges, project, visible_pixels


class ProjectionChecks(unittest.TestCase):
    def test_scale1_raw_mm_equivalent_center_projection(self):
        xyz=np.array([[.1,.2,1.],[-.2,.1,2.]])
        t=np.array([.02,-.03,.01])
        k=[300,301,320,240]
        raw_mm=project(xyz*1000,np.eye(3),t,k,[0]*5)
        metric_equivalent=project(xyz,np.eye(3),t/1000,k,[0]*5)
        np.testing.assert_allclose(raw_mm,metric_equivalent,atol=1e-10)
        # Standard metric geometry should differ when translation is nonzero.
        self.assertGreater(np.max(abs(raw_mm-project(xyz,np.eye(3),t,k,[0]*5))),1)

    def test_distortion_branch_is_material_off_center(self):
        xyz=np.array([[.5,.4,1.]])
        k=[300,300,320,240]
        pinhole=project(xyz,np.eye(3),np.zeros(3),k,[0]*5)
        np.testing.assert_allclose(pinhole,[[470,360]])
        brown=project(xyz,np.eye(3),np.zeros(3),k,[.1,0,0,0,0])
        np.testing.assert_allclose(brown,[[476.15,364.92]])

    def test_hidden_surface_does_not_draw_through_front(self):
        uv=np.array([[10.,10.],[10.,10.],[20.,20.]])
        z=np.array([1.,2.,3.])
        pixels,rejected=visible_pixels(uv,z,uv,z,40,40)
        self.assertEqual(rejected,1)
        self.assertEqual(pixels.tolist(),[[10,10],[20,20]])

    def test_target_camera_depth_controls_visibility(self):
        # A rotated camera has a different depth ordering than source optical Z.
        r=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
        uv,z=project(np.array([[-1.,0.,1.],[-2.,0.,2.]]),r,np.zeros(3),
                     [10,10,0,0],[0]*5,return_depth=True)
        np.testing.assert_allclose(z,[1,2])
        pixels,rejected=visible_pixels(uv,z,uv,z,40,40)
        self.assertEqual(rejected,1)
        self.assertEqual(len(pixels),1)

    def test_both_sides_of_thin_foreground(self):
        z=np.array([[2.,2.,1.,1.,2.,2.]])
        edges,holes=foreground_edges(z,np.ones(z.shape,dtype=bool))
        self.assertEqual(np.flatnonzero(edges).tolist(),[2,3])
        self.assertFalse(holes.any())

    def test_holes_are_not_object_contours(self):
        z=np.array([[1.,0.,1.]])
        edges,holes=foreground_edges(z,z>0)
        self.assertFalse(edges.any())
        self.assertEqual(np.flatnonzero(holes).tolist(),[0,2])

    def test_direction_invariance(self):
        z=np.array([[1.,2.,2.],[1.,1.,2.],[3.,1.,2.]])
        edges,_=foreground_edges(z,z>0)
        flipped,_=foreground_edges(z[::-1,::-1],z[::-1,::-1]>0)
        np.testing.assert_array_equal(edges,flipped[::-1,::-1])
        transposed,_=foreground_edges(z.T,z.T>0)
        np.testing.assert_array_equal(edges,transposed.T)

    def test_project_known_metric_translation(self):
        uv=project(np.array([[0.,0.,1.],[0.,0.,-1.]]),np.eye(3),
                   np.array([.1,0,0]),[100,100,320,240],[0]*5)
        np.testing.assert_allclose(uv,[[330,240]])


if __name__=='__main__':unittest.main()
