import unittest
from capture_rgbd_samples import validate_saved_samples


class CaptureValidationTests(unittest.TestCase):
    def test_empty_and_partial_capture_are_not_success(self):
        for rows in [[], [{'color_file':'a'}], [{'points':20}],
                     [{'color_file':'a','pair_id':0,'stamp':1.},
                      {'points':20,'pair_id':1,'stamp':1.}]]:
            with self.assertRaises(ValueError):validate_saved_samples(rows,True)

    def test_stamp_difference_and_pair_completeness(self):
        rows=[{'color_file':'a','pair_id':0,'stamp':1.},
              {'points':20,'pair_id':0,'stamp':1.01}]
        validate_saved_samples(rows,True)
        rows[1]['stamp']=1.2
        with self.assertRaises(ValueError):validate_saved_samples(rows,True)
