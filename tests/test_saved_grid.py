import tempfile
import unittest
from pathlib import Path
from saved_grid import load_saved_grid

class SavedGridTests(unittest.TestCase):
    def test_complete_grid_flip_thresholds_and_display_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'room.yaml').write_text('image: /home/mi/mapping/room.pgm\nresolution: 0.05\norigin: [1,2,0]\nfree_thresh: 0.196\noccupied_thresh: 0.65\nnegate: 0\n')
            (root/'room.pgm').write_bytes(b'P5\n# saved\n3 2\n255\n'+bytes([0,255,205,255,0,205]))
            grid=load_saved_grid(root,'room')
            self.assertEqual(grid['data'],[0,100,-1,100,0,-1])
            self.assertTrue(grid['display_only'])
            self.assertEqual((grid['width'],grid['height']),(3,2))
            with self.assertRaises(ValueError):load_saved_grid(root,'../room')
            (root/'room.pgm').write_bytes(b'P5\n3 2\n255\n'+bytes([0]))
            with self.assertRaises(ValueError):load_saved_grid(root,'room')
