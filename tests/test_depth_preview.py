import unittest,struct,base64
from types import SimpleNamespace as S
from depth_preview import depth_preview
class DepthPreviewTests(unittest.TestCase):
 def image(self,encoding='16UC1',big=False):
  data=struct.pack(('>' if big else '<')+('4H' if encoding=='16UC1' else '4f'),*([0,1000,2000,5000] if encoding=='16UC1' else [float('nan'),1,2,5]))
  return S(width=2,height=2,encoding=encoding,is_bigendian=big,step=len(data)//2,data=data,header=S(frame_id='depth',stamp=S(sec=100,nanosec=0)))
 def test_metric_endian_and_invalid(self):
  for enc in ['16UC1','32FC1']:
   for big in [False,True]:
    d=depth_preview(self.image(enc,big),100.1)
    self.assertEqual(struct.unpack('<4H',base64.b64decode(d['data_mm_b64'])),(0,1000,2000,0))
    self.assertEqual(d['valid_fraction'],.5)
 def test_stale_and_bad_layout(self):
  with self.assertRaises(ValueError):depth_preview(self.image(),103)
  m=self.image();m.step=1
  with self.assertRaises(ValueError):depth_preview(m,100)
 def test_real_frame_replay(self):
  import json,zlib
  from pathlib import Path
  p=Path('evidence/reconstruction/20261103_103226_878334/frames.jsonl')
  with p.open(encoding='utf-8') as source:f=json.loads(source.readline())
  sec=int(f['stamp'])
  m=S(width=f['width'],height=f['height'],step=f['step'],data=zlib.decompress(base64.b64decode(f['depth_zlib_b64'])),encoding=f['encoding'],is_bigendian=f['bigendian'],header=S(frame_id=f['source_frame'],stamp=S(sec=sec,nanosec=int((f['stamp']-sec)*1e9))))
  d=depth_preview(m,f['stamp']+.1);self.assertLessEqual(d['width'],160);self.assertGreater(d['valid_fraction'],0)
  Path('evidence/navigation_depth_replay.json').write_text(json.dumps(d),encoding='utf-8')
