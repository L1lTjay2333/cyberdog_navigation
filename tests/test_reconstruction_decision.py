import unittest


class PoseDecisionTests(unittest.TestCase):
    def test_healthy_original_with_marginal_improvement_is_kept(self):
        from reconstruction_registration import residual_decision
        self.assertEqual(residual_decision(.0052,.00496),'keep_odometry')

    def test_bad_original_without_useful_correction_is_rejected(self):
        from reconstruction_registration import residual_decision
        self.assertEqual(residual_decision(.014,.0136),'reject')

    def test_good_candidate_is_used_when_gain_is_clear(self):
        from reconstruction_registration import residual_decision
        self.assertEqual(residual_decision(.02,.008),'apply_correction')

    def test_worsened_or_nonfinite_candidate_is_never_accepted(self):
        from reconstruction_registration import residual_decision
        self.assertEqual(residual_decision(.005,.008),'reject')
        self.assertEqual(residual_decision(float('nan'),.001),'reject')

    def test_sub_noise_improvement_does_not_force_pose_change(self):
        from reconstruction_registration import residual_decision
        self.assertEqual(residual_decision(.001,.00095),'keep_odometry')


if __name__=='__main__':unittest.main()
