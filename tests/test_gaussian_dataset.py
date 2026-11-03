import json,tempfile,unittest,base64,zlib
from pathlib import Path
import numpy as np
from test_reconstruction_core import frame
from gaussian_dataset import export


class GaussianDatasetTests(unittest.TestCase):
    def test_duplicate_static_photos_not_multiview(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);session=p/'session';session.mkdir()
            rows=[]
            for i in range(35):
                s=dict(width=2,height=2,step=6,encoding='rgb8',stamp=float(i+1),depth_stamp=float(i+1),
                    data_zlib_b64=base64.b64encode(zlib.compress(bytes([20,30,40]*4))).decode())
                rows.append(frame(float(i+1),color_sample=s))
            (session/'frames.jsonl').write_text('\n'.join(json.dumps(f) for f in rows))
            result=export(session,p/'out')
            self.assertEqual(result['photos'],1)
            self.assertFalse(result['multiview_screen_pass'])
            self.assertFalse(result['training_ready'])

    def test_wrong_pair_time_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);session=p/'session';session.mkdir()
            (session/'frames.jsonl').write_text(json.dumps(frame(color_sample={'stamp':2.,'depth_stamp':1.})))
            with self.assertRaises(ValueError):export(session,p/'out')
