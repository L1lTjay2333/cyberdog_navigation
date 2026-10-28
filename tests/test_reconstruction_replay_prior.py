import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from reconstruction_replay import replay
from reconstruction_core import decode_depth_frame
from test_reconstruction_core import frame


class ReplayPriorTests(unittest.TestCase):
    def test_cumulative_correction_cannot_bypass_five_cm_limit(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'source';source.mkdir()
            (source/'frames.jsonl').write_text('\n'.join(json.dumps(frame(float(i))) for i in (1,2,3))+'\n')
            correction=np.eye(4);correction[0,3]=.03
            with patch('reconstruction_replay.register_clouds',return_value={'accepted':True,'transform':correction.tolist()}):
                result=replay(source,root/'output',refine=True)
            self.assertEqual(result['submaps']['frames'],2)
            last=result['attempts'][-1]['quality']
            self.assertFalse(last['accepted']);self.assertEqual(last['reason'],'cumulative_correction_limit')
            np.testing.assert_array_equal(last['transform'],np.eye(4))

    def test_invalid_target_window_cannot_create_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):replay(temporary,Path(temporary)/'out',target_window=0)
            self.assertFalse((Path(temporary)/'out').exists())

    def test_verified_prior_is_used_for_next_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'source';source.mkdir()
            records=[frame(float(i)) for i in (1,2,3)]
            (source/'frames.jsonl').write_text('\n'.join(map(json.dumps,records))+'\n')
            correction=np.eye(4);correction[0,3]=.01
            qualities=[{'accepted':True,'transform':correction.tolist()},
                       {'accepted':True,'transform':np.eye(4).tolist()}]
            with patch('reconstruction_replay.register_clouds',side_effect=qualities) as registration:
                result=replay(source,root/'output',refine=True)
            expected=decode_depth_frame(records[2])+[.01,0.,0.]
            np.testing.assert_allclose(registration.call_args_list[1].args[0],expected)
            self.assertEqual(result['submaps']['frames'],3)


if __name__=='__main__':unittest.main()
