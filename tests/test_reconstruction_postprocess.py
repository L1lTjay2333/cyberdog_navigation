import base64
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib
import numpy as np
from test_reconstruction_core import frame


class PostprocessTests(unittest.TestCase):
    def test_isolated_spike_is_removed_without_smoothing_other_depths(self):
        from reconstruction_postprocess import filter_depth
        depth=np.ones((7,7));depth[3,3]=2.
        filtered,stats=filter_depth(depth)
        self.assertEqual(filtered[3,3],0.)
        self.assertEqual(stats['isolated_removed'],1)
        np.testing.assert_array_equal(filtered[filtered>0],np.ones(48))
        self.assertEqual(depth[3,3],2.)

    def test_depth_boundary_is_not_averaged_across(self):
        from reconstruction_postprocess import filter_depth
        depth=np.ones((8,8));depth[:,4:]=2.
        filtered,_=filter_depth(depth)
        np.testing.assert_array_equal(filtered,depth)

    def test_voxel_votes_count_frames_not_number_of_points(self):
        from reconstruction_postprocess import VoxelConsensus
        model=VoxelConsensus(.02)
        model.add(np.array([[.001,0,1.]]*100))
        self.assertEqual(model.arrays()[2].tolist(),[1])
        model.add(np.array([[.002,0,1.]]))
        points,keys,support=model.arrays()
        self.assertEqual(support.tolist(),[2])
        self.assertAlmostEqual(points[0,0],.0015)

    def test_single_frame_and_isolated_points_are_not_certified(self):
        from reconstruction_postprocess import select_consensus
        points=np.array([[0.,0.,1.],[.02,0.,1.],[0.,.02,1.],[.02,.02,1.],[5.,5.,5.]])
        keep=select_consensus(points,np.array([2,2,2,1,4]),minimum_neighbors=2)
        np.testing.assert_array_equal(keep,[True,True,True,False,False])

    def test_source_is_unchanged_and_output_is_a_new_archive(self):
        from reconstruction_postprocess import process_capture
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'source';source.mkdir()
            record=frame(width=16,height=16,step=32,intrinsics=[100.,100.,8.,8.])
            record['depth_zlib_b64']=base64.b64encode(zlib.compress(struct.pack('<H',1000)*256)).decode()
            raw='\n'.join(json.dumps(dict(record,stamp=float(i))) for i in (1,2,3))+'\n'
            (source/'frames.jsonl').write_text(raw,encoding='utf-8')
            (source/'summary.json').write_text(json.dumps({'accepted_frames':3,'raw_archive':{'complete':True,'frames':3}}))
            report=process_capture(source,root/'processed')
            self.assertEqual((source/'frames.jsonl').read_text(encoding='utf-8'),raw)
            self.assertGreater(report['processed_points'],0)
            self.assertTrue((root/'processed/points_processed.ply').is_file())
            self.assertEqual(len((root/'processed/frames.jsonl').read_text().splitlines()),3)
            with self.assertRaises(FileExistsError):process_capture(source,root/'processed')

    def test_coordinate_change_is_not_merged(self):
        from reconstruction_postprocess import process_capture
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'source';source.mkdir()
            raw=json.dumps(frame())+'\n'+json.dumps(frame(2.,frame='map'))+'\n'
            (source/'frames.jsonl').write_text(raw)
            with self.assertRaises(ValueError):process_capture(source,root/'out')


if __name__=='__main__':unittest.main()
