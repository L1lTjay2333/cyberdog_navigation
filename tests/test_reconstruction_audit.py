import unittest
import numpy as np


class AuditTests(unittest.TestCase):
    def test_exclusion_reasons_are_distinct_not_all_labeled_noise(self):
        from reconstruction_data_audit import classify_candidates
        points=np.array([[0.,0.,1.],[.01,0.,1.],[1.,1.,1.],[2.,2.,1.]])
        labels,distance=classify_candidates(points,np.array([2,1,1,3]),np.array([True,False,False,False]))
        np.testing.assert_array_equal(labels,[0,1,2,3])
        self.assertAlmostEqual(distance[1],.01)

    def test_depth_discontinuity_is_flagged_without_changing_depth(self):
        from reconstruction_data_audit import edge_flags
        depth=np.ones((8,8));depth[:,4:]=2.
        before=depth.copy();edges=edge_flags(depth)
        self.assertTrue(edges[3,3]);self.assertTrue(edges[3,4]);self.assertFalse(edges[3,1])
        np.testing.assert_array_equal(depth,before)


if __name__=='__main__':unittest.main()
