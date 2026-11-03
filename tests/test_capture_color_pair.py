import ast,base64,zlib,unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace


class ColorPairTests(unittest.TestCase):
    def setup_pair(self):
        source=Path('tools/reconstruction_capture.py').read_text(encoding='utf-8')
        tree=ast.parse(source)
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='color_pair')
        env=dict(colors=deque(),last_color_saved=[0.],counts={'color_pair_misses':0,'color_pairs':0},
                 base64=base64,zlib=zlib,namespace='/test')
        exec(compile(ast.Module(body=[function],type_ignores=[]),'pair','exec'),env)
        return env

    def test_pair_preserves_photo_and_does_not_claim_registration(self):
        e=self.setup_pair();raw=bytes([1,2,3]*4)
        msg=SimpleNamespace(header=SimpleNamespace(frame_id='counter'),width=2,height=2,
                            step=6,encoding='bgr8',data=raw)
        e['colors'].append((10.01,msg));r=e['color_pair'](10.)
        self.assertEqual(zlib.decompress(base64.b64decode(r['data_zlib_b64'])),raw)
        self.assertFalse(r['spatial_registration_verified'])
        self.assertFalse(r['hardware_sync_verified'])
        self.assertIsNone(e['color_pair'](10.5))
        self.assertIsNone(e['color_pair'](11.))
        self.assertEqual(e['counts']['color_pairs'],1)

    def test_missing_color_is_reported_as_missing(self):
        e=self.setup_pair();self.assertIsNone(e['color_pair'](10.))
        self.assertEqual(e['counts']['color_pairs'],0)
        self.assertEqual(e['counts']['color_pair_misses'],1)
