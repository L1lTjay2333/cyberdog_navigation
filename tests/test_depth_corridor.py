import unittest
from depth_corridor import observed_corridor

class CorridorTests(unittest.TestCase):
    def rays(self):
        origin=[.27,0,.115];points=[]
        for y in (-.23,0,.23):
            for z in (.08,.19,.29):
                for _ in range(2):points.append([3,3*y/(1-.27),.115+(z-.115)*(3-.27)/(1-.27)])
        return origin,points
    def test_measured_far_rays_cover_corridor(self):
        origin,points=self.rays();self.assertEqual(observed_corridor(points,origin),1.)
    def test_empty_or_partial_not_clear(self):
        origin,points=self.rays()
        self.assertIsNone(observed_corridor([],origin));self.assertIsNone(observed_corridor(points[:-2],origin))
    def test_near_endpoints_do_not_prove_far_clearance(self):
        self.assertIsNone(observed_corridor([[.5,0,.2]]*100,[.27,0,.115]))
