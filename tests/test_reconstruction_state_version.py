import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from reconstruction_server import Laboratory
from reconstruction_streaming import StreamingModel
from test_reconstruction_core import frame

class StateVersionTests(unittest.TestCase):
    def test_unchanged_poll_skips_preview_and_new_session_invalidates_version(self):
        with tempfile.TemporaryDirectory() as folder:
            lab=Laboratory()
            lab.session='first'
            lab.model=StreamingModel(Path(folder)/'parts')
            lab.model.add(frame(),stride=1)
            lab.model.add(frame(1.5),stride=1)
            initial=lab.report()
            self.assertTrue(initial['points'])
            with patch.object(lab.model,'preview',side_effect=AssertionError('redundant preview')):
                same=lab.report(since=initial['model_version'])
                self.assertNotIn('points',same)
                light=lab.report(False)
                self.assertNotIn('points',light)
            lab.session='second'
            self.assertIn('points',lab.report(since=initial['model_version']))
            lab.model.add(frame(2),stride=1)
            self.assertNotEqual(lab.report()['model_version'],initial['model_version'])
