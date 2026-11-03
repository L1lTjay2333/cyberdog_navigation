import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class WorkbenchTests(unittest.TestCase):
    def test_invalid_mode_and_frame_cannot_launch_worker(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            bench=Workbench(Path(temporary))
            with patch('reconstruction_workbench.subprocess.Popen') as process:
                for mode,first in [('invalid',1),('local',0),('local',True),('local',900)]:
                    with self.assertRaises(ValueError):bench.start('20261028_163545_462434',mode,first)
                process.assert_not_called()

    def setup_capture(self,root,name='20261028_163545_462434'):
        folder=root/'evidence/reconstruction'/name;folder.mkdir(parents=True)
        (folder/'frames.jsonl').write_text('{}\n')
        (folder/'summary.json').write_text(json.dumps({'accepted_frames':44,'stable_points':66799,'raw_archive':{'frames':44,'complete':True},'error':'capacity'}))
        return folder

    def test_lists_only_nonempty_bounded_archived_sessions(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.setup_capture(root)
            empty=self.setup_capture(root,'20261028_163519_314736')
            (empty/'frames.jsonl').write_text('')
            self.assertEqual(len(Workbench(root).sessions()),1)

    def test_invalid_session_cannot_launch_any_process(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            bench=Workbench(Path(temporary))
            with patch('reconstruction_workbench.subprocess.Popen') as process:
                for name in ['../../config','C:/Windows','not_a_session']:
                    with self.assertRaises(ValueError):bench.start(name)
            process.assert_not_called()

    def test_only_one_worker_can_run_and_cancel_owns_that_process(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=self.setup_capture(root);before=(source/'frames.jsonl').read_bytes()
            bench=Workbench(root)
            with patch('reconstruction_workbench.subprocess.Popen') as process:
                process.return_value.poll.return_value=None
                first=bench.start(source.name)
                with self.assertRaises(ValueError):bench.start(source.name)
                bench.cancel()
                process.return_value.terminate.assert_called_once()
            self.assertEqual((source/'frames.jsonl').read_bytes(),before)
            self.assertEqual(bench.snapshot()['status'],'cancelled')

    def test_failed_worker_cannot_publish_a_successful_result(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);folder=self.setup_capture(root);bench=Workbench(root)
            with patch('reconstruction_workbench.subprocess.Popen') as process:
                process.return_value.poll.return_value=None;bench.start(folder.name)
                process.return_value.poll.return_value=1
                state=bench.snapshot()
            self.assertEqual(state['status'],'failed')
            self.assertFalse(state.get('results'))

    def test_result_names_cannot_escape_output_directory(self):
        from reconstruction_workbench import Workbench
        with tempfile.TemporaryDirectory() as temporary:
            bench=Workbench(Path(temporary))
            for run,name in [('bad','observed.bin'),('20261028_123456_123456','../../config/scope.json')]:
                with self.assertRaises(ValueError):bench.result_file(run,name)

    def test_http_start_requires_same_origin(self):
        from reconstruction_workbench import Handler
        handler=object.__new__(Handler);handler.path='/api/start'
        data=json.dumps({'session':'20261028_163545_462434'}).encode()
        handler.headers={'Origin':'https://untrusted.invalid','Content-Type':'application/json','Content-Length':str(len(data))}
        handler.rfile=io.BytesIO(data);output=[]
        handler.send_json=lambda value,status=200:output.append((status,value))
        with patch('reconstruction_workbench.BENCH') as bench:handler.do_POST()
        bench.start.assert_not_called();self.assertEqual(output[0][0],400)


if __name__=='__main__':unittest.main()
