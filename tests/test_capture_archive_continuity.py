import tempfile
import unittest
from pathlib import Path
from reconstruction_server import Laboratory
from reconstruction_streaming import StreamingModel
from reconstruction_archive import FrameArchive
from test_reconstruction_core import frame

class ArchiveContinuity(unittest.TestCase):
    def test_jump_preserves_future_frames_without_false_stitching(self):
        with tempfile.TemporaryDirectory() as folder:
            lab=Laboratory()
            lab.model=StreamingModel(Path(folder)/'submaps')
            lab.raw_archive=FrameArchive(Path(folder)/'frames.jsonl')
            for t,x in [(1.,0.),(1.2,.01),(1.4,2.),(1.6,2.01)]:
                lab.accept_record(frame(t,translation=[x,0.,0.]))
            self.assertEqual(lab.received_frames,4)
            self.assertEqual(lab.model.accepted_frames,2)
            self.assertTrue(lab.fusion_paused)
            self.assertTrue(lab.raw_archive.stats(4)['complete'])
            self.assertEqual(len(lab.raw_archive.path.read_text().splitlines()),4)
