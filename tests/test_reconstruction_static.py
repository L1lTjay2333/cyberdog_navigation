import unittest
import numpy as np
from test_reconstruction_core import frame
from reconstruction_static import ConsensusStaticModel


class StaticConsensusIntegrationTests(unittest.TestCase):
    def test_majority_rejects_spike_and_preserves_edge(self):
        m = ConsensusStaticModel(voxel=.001)
        for i, value in enumerate([1000, 1002, 1800, 999, 1001]):
            m.add(frame(float(i+1), values=(value, 2000, value, 2000)), stride=1)
            if i < 4:
                self.assertEqual(m.points(), [])
        zs = np.array(m.points())[:, 2]
        self.assertEqual(len(zs), 4)
        np.testing.assert_allclose(sorted(zs), [1.0005, 1.0005, 2., 2.], atol=1e-6)
        self.assertEqual(m.stats()['consensus_updates'], 1)

    def test_no_majority_leaves_hole_and_no_stale_surface(self):
        m = ConsensusStaticModel()
        for i in range(5):
            m.add(frame(float(i+1)), stride=1)
        self.assertTrue(m.points())
        for i, value in enumerate([1200, 1400, 1600, 1800, 2000]):
            m.add(frame(float(i+6), values=(value,)*4), stride=1)
        self.assertEqual(m.points(), [])
        self.assertEqual(m.stats()['consensus_pixels'], 0)

    def test_pose_calibration_and_gap_reject_before_mutation(self):
        for changed in [frame(2., translation=[.04,0.,0.]),
                        frame(2., intrinsics=[101.,100.,.5,.5]),
                        frame(2., source_frame='other'), frame(5.)]:
            with self.subTest(changed=changed):
                m = ConsensusStaticModel();m.add(frame(),stride=1)
                with self.assertRaises(ValueError):m.add(changed,stride=1)
                self.assertEqual(m.accepted_frames, 1)
                self.assertEqual(len(m.history), 1)

    def test_duplicate_does_not_vote(self):
        m = ConsensusStaticModel();m.add(frame(),stride=1)
        for _ in range(6):self.assertFalse(m.add(frame(),stride=1))
        self.assertEqual(m.stats()['temporal_window_frames'],1)
        self.assertEqual(m.points(),[])

    def test_small_axial_motion_is_compensated_before_consensus(self):
        m=ConsensusStaticModel(voxel=.001)
        for i in range(5):
            # A fixed plane at world Z=1 while camera moves forward 5 mm/frame.
            m.add(frame(float(i+1),values=(1000-i*5,)*4,
                        translation=[0.,0.,i*.005]),stride=1)
        np.testing.assert_allclose(np.asarray(m.points())[:,2],1.,atol=1e-6)

    def test_server_selects_consensus_only_for_static(self):
        from reconstruction_server import Laboratory
        from reconstruction_core import MovingModel
        lab=Laboratory();lab.begin_capture(30,'static')
        self.assertIsInstance(lab.model,ConsensusStaticModel)
        lab.begin_capture(30,'moving')
        self.assertIsInstance(lab.model,MovingModel)


if __name__ == '__main__':unittest.main()
