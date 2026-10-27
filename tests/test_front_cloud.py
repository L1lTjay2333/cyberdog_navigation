import struct
import types
import unittest
from front_cloud import forward_cloud_distance,forward_points_distance

def cloud(points):
    fields=[types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    return types.SimpleNamespace(width=len(points),height=1,point_step=12,row_step=len(points)*12,
        fields=fields,is_bigendian=False,data=b''.join(struct.pack('<fff',*p) for p in points))

class FrontCloudTests(unittest.TestCase):
    def test_image_periphery_invalid_but_corridor_observed(self):
        rays=[[2,y,z] for y in (-.5,0,.5) for z in (.16,.38,.60) for _ in range(2)]
        distance,_=forward_points_distance(rays,1200,len(rays)/1200.,[0,0,0])
        self.assertEqual(distance,1)
        with self.assertRaises(ValueError):
            forward_points_distance(rays[:-2],1200,len(rays[:-2])/1200.,[0,0,0])
    def test_image_near_point_not_hidden_by_invalid_periphery(self):
        distance,_=forward_points_distance([[.4,0,.3]],1200,1/1200.,[0,0,0])
        self.assertEqual(distance,.4)
    def test_valid_native_cloud(self):
        distance,count=forward_cloud_distance(cloud([[2,0,.3]]*1000),[0,0,0],[0,0,0,1])
        self.assertEqual(distance,2);self.assertEqual(count,1000)
    def test_near_obstacle_retained(self):
        distance,_=forward_cloud_distance(cloud([[.4,0,.3]]*1000),[0,0,0],[0,0,0,1])
        self.assertAlmostEqual(distance,.4)
    def test_measured_far_wall_is_not_missing_depth(self):
        distance,count=forward_cloud_distance(cloud([[6,0,.3]]*1000),[0,0,0],[0,0,0,1])
        self.assertEqual(distance,6);self.assertEqual(count,1000)
    def test_missing_coverage_rejected(self):
        with self.assertRaises(ValueError):forward_cloud_distance(cloud([[0,0,0]]*1000),[0,0,0],[0,0,0,1])
    def test_empty_roi_not_treated_as_free(self):
        with self.assertRaises(ValueError):forward_cloud_distance(cloud([[2,2,.3]]*1000),[0,0,0],[0,0,0,1])
    def test_sparse_periphery_requires_complete_measured_forward_bins(self):
        rays=[[2,y,z] for y in (-.5,0,.5) for z in (.16,.38,.60) for _ in range(2)]
        distance,_=forward_cloud_distance(cloud(rays+[[0,0,0]]*1000),[0,0,0],[0,0,0,1])
        self.assertEqual(distance,1,'Sparse frames yield only the observed bound, not unlimited free space')
    def test_sparse_missing_bin_still_rejected(self):
        rays=[[2,y,z] for y in (-.5,0,.5) for z in (.16,.38,.60) for _ in range(2)]
        with self.assertRaises(ValueError):forward_cloud_distance(cloud(rays[:-2]+[[0,0,0]]*1000),[0,0,0],[0,0,0,1])
    def test_sparse_near_obstacle_is_not_discarded(self):
        distance,_=forward_cloud_distance(cloud([[.4,0,.3]]+[[0,0,0]]*1000),[0,0,0],[0,0,0,1])
        self.assertAlmostEqual(distance,.4)
