import unittest,tempfile,json
from pathlib import Path
from live_cloud_archive import LiveCloudArchive
class ArchiveTest(unittest.TestCase):
 def test_save_restore_and_reject(self):
  with tempfile.TemporaryDirectory() as d:
   a=LiveCloudArchive(d)
   def s(stamp,aligned=True,frame='laser_odom'):
    return {'task':{'status':5},'emitted_unix':stamp+.1,'cloud':{'aligned':aligned,'frame':frame,'stamp':stamp,'points_xyz':[[1.,2.,3.]]}}
   a.add(s(10,False),'room');self.assertFalse(a.path.exists())
   a.add(s(10),'room');a.add(s(10),'room')
   self.assertEqual(len(next(Path(d).glob('session_*/frames.jsonl')).read_text().splitlines()),1)
   b=LiveCloudArchive(d);self.assertEqual(b.read()['points_xyz'],[[1.,2.,3.]])
   a.saved=0;a.add(s(11),'room2');self.assertEqual(a.read()['key'],'room2:laser_odom')
   self.assertTrue(next(Path(d).glob('session_*/map.json')).exists())
