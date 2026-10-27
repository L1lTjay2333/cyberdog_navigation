import sys
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from depth_geometry import depth_xyz,manual_sample_stride

class DepthGeometryTests(unittest.TestCase):
    def test_manual_resolution_sampling_not_always_invalid(self):
        for width,height in ((320,240),(640,480),(1280,720)):
            stride=manual_sample_stride(width,height)
            samples=len(range(0,width,stride))*len(range(0,height,stride))
            self.assertGreaterEqual(samples,1000)
            self.assertLess(samples,1600)
    def inputs(self, big=False):
        h=NS(frame_id='depth_optical')
        image=NS(width=2,height=1,header=h,encoding='16UC1',is_bigendian=big,
                 step=6,data=struct.pack(('>' if big else '<')+'HH',2000,0)+b'\0\0')
        info=NS(width=2,height=1,header=h,p=[100.,0.,0.,0.,0.,100.,0.,0.,0.,0.,1.,0.])
        return image,info
    def test_meter_scale_and_invalid_zero(self):
        pts,total=depth_xyz(*self.inputs(),stride=1)
        self.assertEqual(pts,[[0.,0.,2.]])
        self.assertEqual(total,2)
    def test_big_endian_padding(self):
        self.assertEqual(depth_xyz(*self.inputs(True),stride=1)[0],[[0.,0.,2.]])
    def test_bad_layout(self):
        image,info=self.inputs();image.data=b''
        with self.assertRaises(ValueError):depth_xyz(image,info)
    def test_bad_frame(self):
        image,info=self.inputs();info.header=NS(frame_id='other')
        with self.assertRaises(ValueError):depth_xyz(image,info)
    def test_rgb_rejected(self):
        image,info=self.inputs();image.encoding='rgb8'
        with self.assertRaises(ValueError):depth_xyz(image,info)

if __name__=='__main__':unittest.main()
