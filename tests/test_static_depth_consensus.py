import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from static_depth_consensus import consensus


class ConsensusTest(unittest.TestCase):
    def pixel(self,values):return consensus(np.array(values,dtype=float)[:,None,None])

    def test_majority_rejects_large_outlier(self):
        d,n,k=self.pixel([1,1.003,1.006,3])
        self.assertAlmostEqual(d[0,0],1.003);self.assertEqual(n[0,0],3);self.assertEqual(k,3)

    def test_no_phantom_middle_between_equal_surfaces(self):
        d,n,k=self.pixel([1,1,2,2]);self.assertTrue(np.isnan(d[0,0]))

    def test_missing_values_cannot_vote(self):
        d,_,_=self.pixel([1,1.001,0,np.nan]);self.assertTrue(np.isnan(d[0,0]))

    def test_edges_not_spatially_blurred(self):
        frames=np.repeat(np.array([[[1,2],[1,2]]],dtype=float),4,axis=0)
        d,_,_=consensus(frames);np.testing.assert_array_equal(d,frames[0])

    def test_later_sorted_cluster_can_win(self):
        d,n,_=self.pixel([.3,2,2.004,2.006]);self.assertAlmostEqual(d[0,0],2.004);self.assertEqual(n[0,0],3)


if __name__=='__main__':unittest.main()
