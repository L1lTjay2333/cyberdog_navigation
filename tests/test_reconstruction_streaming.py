import json
import io
import shutil
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from reconstruction_streaming import StreamingModel
from reconstruction_core import decode_depth_frame
from test_reconstruction_core import frame
import reconstruction_server as server


class StreamingTests(unittest.TestCase):
    def test_capture_service_rolls_submaps_without_real_robot(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);(root/'tools').mkdir()
            for name in ('reconstruction_core.py','reconstruction_capture.py'):
                shutil.copyfile(server.ROOT/'tools'/name,root/'tools'/name)
            records=[]
            for i in range(8):
                for j in range(2):
                    stamp=1+i*.5+j*.25
                    records.append(dict(frame(stamp,translation=[i*.025,0.,0.]),event='depth',tf_stamp=stamp))
            process=SimpleNamespace(stdin=io.StringIO(),stdout=iter(json.dumps(r)+'\n' for r in records),
                                    wait=lambda timeout:0,poll=lambda:0)
            class SmallStreaming(StreamingModel):
                def __init__(self,folder):super().__init__(folder,max_voxels=2)
            lab=server.Laboratory();lab.model=server.MovingModel();lab.session='mock_stream';lab.active=True
            with patch.object(server,'ROOT',root),patch.object(server,'StreamingModel',SmallStreaming),patch.object(server.subprocess,'Popen',return_value=process):
                lab.capture(10)
            self.assertFalse(lab.active);self.assertIsNone(lab.error)
            folder=root/'evidence/reconstruction/mock_stream'
            summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['accepted_frames'],16)
            self.assertTrue(summary['streaming_submaps'])
            self.assertGreater(summary['sealed_parts'],1)
            self.assertTrue(summary['raw_archive']['complete'])
            ply=(folder/'model.ply').read_text()
            self.assertIn('element vertex %d\n'%summary['stable_points'],ply)

    def test_longer_moving_session_is_bounded_and_static_limit_unchanged(self):
        lab=server.Laboratory()
        with patch.object(server.threading,'Thread') as thread:
            lab.start(300,'moving')
            thread.return_value.start.assert_called_once()
        self.assertTrue(lab.active)
        with self.assertRaises(ValueError):server.Laboratory().start(301,'moving')
        with self.assertRaises(ValueError):server.Laboratory().start(61,'static')

    def test_rollover_continues_preserves_points_and_bounded_preview(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = StreamingModel(Path(temporary)/'parts',max_voxels=8)
            for i in range(10):
                f=frame(1+i*.5,translation=[i*.025,0.,0.])
                model.add(f,stride=1)
                model.add(dict(f,stamp=f['stamp']+.25),stride=1)
                self.assertLessEqual(len(model.store.cells),8)
            self.assertEqual(model.accepted_frames,20)
            self.assertGreater(len(model.store.parts),1)
            self.assertLessEqual(len(model.preview(limit=6)),6)
            count,batches=model.export_snapshot()
            self.assertEqual(sum(len(p) for p in batches()),count)
            self.assertGreater(count,0)
            self.assertTrue(all(p['voxels']<=8 for p in model.store.parts))

    def test_finish_restore_and_full_export_match(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);model=StreamingModel(root/'parts',max_voxels=8)
            for i in range(6):model.add(frame(1+i*.5,translation=[i*.025,0.,0.]),stride=1)
            model.finish()
            lab=server.Laboratory();lab.model=model;lab.session='roundtrip'
            summary=lab.report(False);before=lab.ply()
            restored=StreamingModel.restore(root/'parts',summary,frame())
            lab.model=restored
            self.assertEqual(lab.report()['stable_points'],summary['stable_points'])
            self.assertEqual(lab.ply(),before)
            self.assertLessEqual(len(lab.report()['points']),30000)

    def test_export_snapshot_remains_consistent_while_more_frames_arrive(self):
        with tempfile.TemporaryDirectory() as temporary:
            model=StreamingModel(Path(temporary)/'parts',max_voxels=8)
            model.add(frame(),stride=1);model.add(frame(1.5),stride=1)
            count,batches=model.export_snapshot()
            for i in range(1,7):model.add(frame(1.5+i*.5,translation=[i*.025,0.,0.]),stride=1)
            self.assertEqual(sum(len(p) for p in batches()),count)

    def test_frame_change_jump_duplicate_and_time_gap_still_guarded(self):
        with tempfile.TemporaryDirectory() as temporary:
            model=StreamingModel(Path(temporary)/'parts');model.add(frame(),stride=1)
            before=model.stats()
            for f in [frame(1.5,frame='map'),frame(1.1,translation=[1.,0.,0.]),frame(5.)]:
                with self.assertRaises(ValueError):model.add(f,stride=1)
                self.assertEqual(model.stats(),before)
            self.assertFalse(model.add(frame(),stride=1))

    def test_room_motion_not_limited_to_previous_three_meter_trial(self):
        with tempfile.TemporaryDirectory() as temporary:
            model=StreamingModel(Path(temporary)/'parts')
            for i in range(35):model.add(frame(1+i*.5,translation=[i*.1,0.,0.]),stride=1)
            self.assertGreater(model.travel_m,3.)
            self.assertIsNone(model.stats()['max_travel_m'])

    def test_preview_is_smaller_than_full_export_without_false_count(self):
        with tempfile.TemporaryDirectory() as temporary:
            model=StreamingModel(Path(temporary)/'parts',max_voxels=8)
            for i in range(8):
                f=frame(1+i*.5,translation=[i*.025,0.,0.])
                model.add(f,stride=1);model.add(dict(f,stamp=f['stamp']+.25),stride=1)
            self.assertGreater(model.stable_count(),len(model.preview(limit=1)))
            lab=server.Laboratory();lab.model=model
            count,batches=model.export_snapshot();body=lab.ply().decode()
            self.assertIn('element vertex %d\n'%count,body)
            self.assertEqual(len(body.split('end_header\n')[1].splitlines()),count)

    def test_streaming_restore_via_service_never_connects_robot(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);folder=root/'evidence/reconstruction/stream';folder.mkdir(parents=True)
            model=StreamingModel(folder/'submaps',max_voxels=8)
            model.add(frame(),stride=1);model.add(frame(1.5),stride=1);model.finish()
            old=server.Laboratory();old.model=model;old.session='stream'
            (folder/'frame_01.json').write_text(json.dumps(frame()),encoding='utf-8')
            (folder/'summary.json').write_text(json.dumps(old.report(False)),encoding='utf-8')
            (folder/'model.ply').write_bytes(old.ply())
            lab=server.Laboratory()
            with patch.object(server,'ROOT',root),patch.object(server.subprocess,'Popen') as process:
                self.assertTrue(lab.restore_latest())
            process.assert_not_called();self.assertIsInstance(lab.model,StreamingModel)
            self.assertEqual(lab.report()['stable_points'],4)


if __name__=='__main__':unittest.main()
