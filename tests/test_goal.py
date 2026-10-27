import unittest
from cyberdog_navigation.goal import MapGoal

class GoalTests(unittest.TestCase):
    def test_map_origin(self):
        self.assertEqual(MapGoal(-0.75, 2.25, 0).validate([[0,0],[0,0]], 0.5, (-1,2)), (0,0))

    def test_negative_outside_does_not_round_into_map(self):
        with self.assertRaises(ValueError):
            MapGoal(-0.001, 0, 0).validate([[0]], 1)

    def test_unknown_and_obstacle_rejected(self):
        for cell in [-1, 100]:
            with self.assertRaises(ValueError):
                MapGoal(0, 0, 0).validate([[cell]], 1)

    def test_invalid_frame_and_nan(self):
        for goal in [MapGoal(0,0,0,'odom'), MapGoal(float('nan'),0,0)]:
            with self.assertRaises(ValueError):
                goal.validate([[0]], 1)

if __name__ == '__main__':
    unittest.main()
