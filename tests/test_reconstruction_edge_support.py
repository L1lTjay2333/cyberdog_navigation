import unittest
import numpy as np
from test_reconstruction_core import frame
from reconstruction_edge_support import view_evidence,confirm_edges


class EdgeSupportTests(unittest.TestCase):
    def test_two_views_can_confirm_thin_surface(self):
        p=np.array([[0.,0.,1.]])
        keep,votes,conflicts=confirm_edges(p,[(frame(2.),np.ones((2,2))),(frame(3.),np.ones((2,2)))])
        self.assertTrue(keep[0]);self.assertEqual(votes[0],2);self.assertEqual(conflicts[0],0)

    def test_one_view_does_not_confirm(self):
        keep,_,_=confirm_edges(np.array([[0.,0.,1.]]),[(frame(2.),np.ones((2,2)))])
        self.assertFalse(keep[0])

    def test_occlusion_is_unknown_not_free_space_conflict(self):
        agree,conflict=view_evidence(np.array([[0.,0.,1.]]),frame(),np.full((2,2),.5))
        self.assertFalse(agree[0]);self.assertFalse(conflict[0])

    def test_free_space_conflict_blocks_confirmation(self):
        p=np.array([[0.,0.,1.]])
        neighbors=[(frame(2.),np.ones((2,2))),(frame(3.),np.ones((2,2))),(frame(4.),np.full((2,2),2.))]
        keep,_,conflicts=confirm_edges(p,neighbors)
        self.assertFalse(keep[0]);self.assertEqual(conflicts[0],1)

    def test_duplicate_timestamp_cannot_inflate_support(self):
        with self.assertRaises(ValueError):confirm_edges(np.array([[0.,0.,1.]]),[(frame(),np.ones((2,2)))]*2)

    def test_out_of_view_never_confirms(self):
        a,c=view_evidence(np.array([[10.,0.,1.],[0.,0.,-1.]]),frame(),np.ones((2,2)))
        self.assertFalse(a.any());self.assertFalse(c.any())

    def test_world_point_is_projected_through_camera_rotation(self):
        q=[0.,2**-.5,0.,2**-.5]
        a,c=view_evidence(np.array([[1.,0.,0.]]),frame(quaternion=q),np.ones((2,2)))
        self.assertTrue(a[0]);self.assertFalse(c[0])
