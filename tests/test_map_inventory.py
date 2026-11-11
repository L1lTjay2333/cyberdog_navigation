import sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from map_inventory import archive_other_maps,assert_active_map,stage_previous_map,finish_map_rotation
class InventoryTests(unittest.TestCase):
    name='computer_room_20261009_110107'
    def prepare(self,parent):
        folder=Path(parent)/'mapping';folder.mkdir()
        for name in (self.name+'.pgm',self.name+'.yaml',self.name+'.json','graph.pbstream','pose.txt','old.pgm','old.yaml','old.json'):
            (folder/name).write_bytes(name.encode())
        return folder
    def test_backups_and_archives_old_files_only(self):
        with tempfile.TemporaryDirectory() as d:
            folder=self.prepare(d);stage_previous_map(folder,lambda *a,**k:None);result=archive_other_maps(folder,self.name)
            self.assertEqual(len(result['archived']),3)
            self.assertEqual((Path(result['backup'])/'mapping/old.pgm').read_bytes(),b'old.pgm')
            self.assertTrue((Path(result['backup'])/'mapping/old.pgm').exists())
            self.assertEqual((folder/'graph.pbstream').read_bytes(),b'graph.pbstream')
            assert_active_map(folder,self.name)
    def test_multiple_maps_refused_before_localization(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(RuntimeError):assert_active_map(self.prepare(d),self.name)
    def test_missing_current_map_changes_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            folder=self.prepare(d);(folder/(self.name+'.json')).unlink()
            with self.assertRaises(RuntimeError):archive_other_maps(folder,self.name)
            self.assertTrue((folder/'old.pgm').exists())
    def test_single_map_is_noop(self):
        with tempfile.TemporaryDirectory() as d:
            folder=self.prepare(d);stage_previous_map(folder,lambda *a,**k:None);archive_other_maps(folder,self.name)
            self.assertIsNone(archive_other_maps(folder,self.name)['backup'])

    def test_repeated_start_and_success_keep_two(self):
        with tempfile.TemporaryDirectory() as d:
            folder=self.prepare(d)
            stage_previous_map(folder,lambda *a,**k:None)
            stage_previous_map(folder,lambda *a,**k:None)
            finish_map_rotation(folder,self.name)
            self.assertTrue((Path(d)/'computer_map_previous/mapping/old.pgm').exists())
            stage_previous_map(folder,lambda *a,**k:None)
            self.assertTrue((Path(d)/'computer_map_previous/mapping/old.pgm').exists())
            self.assertTrue((Path(d)/'computer_map_pending/mapping'/ (self.name+'.pgm')).exists())
            next_name='computer_room_20261105_160000'
            for suffix in ('.pgm','.yaml','.json'):
                (folder/(next_name+suffix)).write_bytes(b'new')
            finish_map_rotation(folder,next_name)
            self.assertEqual(sorted(p.name for p in Path(d).iterdir()),['computer_map_previous','mapping'])
            self.assertFalse((folder/(self.name+'.pgm')).exists())
            self.assertTrue((Path(d)/'computer_map_previous/mapping'/ (self.name+'.pgm')).exists())

    def test_failed_save_preserves_snapshot(self):
        with tempfile.TemporaryDirectory() as d:
            folder=self.prepare(d)
            stage_previous_map(folder,lambda *a,**k:None)
            (folder/(self.name+'.json')).unlink()
            with self.assertRaises(RuntimeError):
                finish_map_rotation(folder,self.name)
            self.assertTrue((Path(d)/'computer_map_pending/mapping'/ (self.name+'.json')).exists())

if __name__=='__main__':unittest.main()
