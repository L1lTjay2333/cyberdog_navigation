import base64,json,tempfile,unittest,zlib
from pathlib import Path
import numpy as np
from reconstruction_color import photo,colored_cloud,calibration,run_color
from test_reconstruction_core import frame


class ColorGeometryTests(unittest.TestCase):
    def sample(self):
        y,x=np.mgrid[:40,:40];rgb=np.stack((x*6,y*6,(x+y)*3),axis=-1).astype('uint8')
        s=dict(width=40,height=40,step=120,encoding='rgb8',stamp=1.,depth_stamp=1.,
               data_zlib_b64=base64.b64encode(zlib.compress(rgb.tobytes())).decode())
        f=frame(width=40,height=40,step=80,intrinsics=[100.,100.,20.,20.],
                depth_zlib_b64=base64.b64encode(zlib.compress(np.full((40,40),1000,dtype='<u2').tobytes())).decode(),color_sample=s)
        c=dict(depth_frame=f['source_frame'],depth_intrinsics=f['intrinsics'],
               depth_to_color=np.eye(4).tolist(),color_intrinsics=f['intrinsics'],
               distortion=[0.]*5,color_size=[40,40])
        return rgb,s,f,c

    def test_photo_decode_and_registered_colors(self):
        rgb,s,f,c=self.sample();np.testing.assert_array_equal(photo(s),rgb)
        cloud=colored_cloud(f,c)
        points=np.asarray(cloud.points);colors=np.asarray(cloud.colors)
        u=np.rint(points[:,0]*100+20).astype(int);v=np.rint(points[:,1]*100+20).astype(int)
        np.testing.assert_allclose(colors,rgb[v,u]/255.)

    def test_time_and_intrinsics_mismatch_rejected(self):
        _,s,f,c=self.sample();s['stamp']=1.2
        with self.assertRaises(ValueError):colored_cloud(f,c)
        s['stamp']=1.;c['depth_intrinsics']=[101.,100.,20.,20.]
        with self.assertRaises(ValueError):colored_cloud(f,c)

    def test_unverified_calibration_and_old_depth_only_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'c.json').write_text('{}')
            with self.assertRaises(ValueError):calibration(p/'c.json')
            out=p/'output';out.mkdir()
            with self.assertRaisesRegex(ValueError,'rgbd_calibration'):run_color(p,out)

    def test_truncated_or_trailing_photo_rejected(self):
        _,s,_,_=self.sample();s['data_zlib_b64']=base64.b64encode(zlib.compress(b'bad')).decode()
        with self.assertRaises(ValueError):photo(s)
