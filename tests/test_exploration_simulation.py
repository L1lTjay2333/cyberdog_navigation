import unittest
from simulate_exploration import simulate

class IdealizedExplorationTests(unittest.TestCase):
    def test_unknown_straight_reaches_in_planar_model(self):
        r=simulate();self.assertEqual(r['result'],'ARRIVED');self.assertEqual(r['phases'],['FRONTIER_ROUTE','GOAL_ROUTE'])
    def test_detour_longer_than_fixed_straight_distance(self):
        r=simulate(detour=True);self.assertEqual(r['result'],'ARRIVED');self.assertGreater(r['path_length_m'],r['straight_distance_m'])
    def test_known_full_wall_stops_no_route(self):
        r=simulate(closed=True);self.assertEqual(r['result'],'NO_ROUTE');self.assertEqual(r['path_length_m'],0)
