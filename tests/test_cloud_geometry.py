import math
import struct
import types
import unittest
from unittest.mock import patch
import cloud_geometry
from cloud_geometry import decode_xyz,transform_xyz

class CloudTests(unittest.TestCase):
    def test_accelerated_decoder_same_samples_padding_and_endian(self):
        for endian in ('<','>'):
            cloud=self.cloud(endian);cloud.height=1200
            cloud.data=cloud.data*600
            accelerated=decode_xyz(cloud,1501)
            with patch.object(cloud_geometry,'_np',None):
                reference=decode_xyz(cloud,1501)
            self.assertEqual(accelerated,reference)
    def test_accelerated_transform_matches_python(self):
        points=[[1.,2.,3.],[-2.,5.,.1]]*600
        accelerated=transform_xyz(points,[.2,.3,.4],[.1,.2,.3,.8])
        with patch.object(cloud_geometry,'_np',None):
            reference=transform_xyz(points,[.2,.3,.4],[.1,.2,.3,.8])
        for p,q in zip(accelerated,reference):
            for a,b in zip(p,q):self.assertAlmostEqual(a,b,places=12)
    def cloud(self,endian='<'):
        fields=[types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
        return types.SimpleNamespace(width=1,height=2,point_step=12,row_step=16,fields=fields,is_bigendian=endian=='>',
            data=struct.pack(endian+'fff',1,2,3)+b'PAD!'+struct.pack(endian+'fff',4,5,6)+b'PAD!')
    def test_endian_and_padding(self):
        for endian in ('<','>'):self.assertEqual(decode_xyz(self.cloud(endian)),[[1,2,3],[4,5,6]])
    def test_invalid_layout(self):
        cloud=self.cloud();cloud.row_step=11
        with self.assertRaises(ValueError):decode_xyz(cloud)
    def test_nonfinite_and_sampling(self):
        cloud=self.cloud();cloud.data=struct.pack('<fff',float('nan'),2,3)+b'PAD!'+struct.pack('<fff',4,5,6)+b'PAD!'
        self.assertEqual(decode_xyz(cloud),[[4,5,6]])
        self.assertEqual(decode_xyz(self.cloud(),1),[[1,2,3]])
    def test_sensor_rotation_translation(self):
        point=transform_xyz([[1,0,0]],[2,3,4],[0,0,math.sqrt(.5),math.sqrt(.5)])[0]
        for actual,expected in zip(point,[2,4,4]):self.assertAlmostEqual(actual,expected)
        with self.assertRaises(ValueError):transform_xyz([[1,2,3]],[0,0,0],[0,0,0,0])

if __name__=='__main__':unittest.main()
