import unittest
import base64,zlib
import numpy as np
from test_reconstruction_core import frame
from reconstruction_target import TargetBox
from reconstruction_postprocess import read_depth


def scene(stamp=1.,translation=None):
    z=np.full((20,20),1000,dtype='<u2');z[:,0:5]=3000
    return frame(stamp,width=20,height=20,step=40,intrinsics=[100.,100.,10.,10.],
                 translation=translation or [0.,0.,0.],
                 depth_zlib_b64=base64.b64encode(zlib.compress(z.tobytes())).decode())


class TargetTests(unittest.TestCase):
    def test_background_masked_and_target_anchor_fixed(self):
        box=TargetBox([.5,.5,.5]);out=box.apply(scene());center=box.center.copy()
        _,z=read_depth(out)
        self.assertTrue((z[:,:5]==0).all());self.assertTrue((z[:,5:]==1).all())
        box.apply(scene(2.,[.1,0.,0.]))
        np.testing.assert_array_equal(center,box.center)
        self.assertEqual(out['target_box']['frame'],'vodom')

    def test_bad_size_and_invalid_depth_rejected(self):
        for size in ([0,1,1],[1,1],[float('nan'),1,1],[4,1,1]):
            with self.assertRaises(ValueError):TargetBox(size)
        bad=scene();bad['depth_zlib_b64']='broken'
        with self.assertRaises(Exception):TargetBox([1,1,1]).apply(bad)
