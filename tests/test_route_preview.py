import unittest
from route_preview import preview
class RouteTests(unittest.TestCase):
    def grid(self):return {'width':20,'height':20,'resolution':0.1,'frame':'map','origin':{'x':0,'y':0,'qz':0,'qw':1},'data':[0]*400}
    def test_routes_around_wall(self):
        g=self.grid()
        for y in range(5,15):g['data'][y*20+10]=100
        route=preview(g,{'x':0.5,'y':1,'frame':'map'},1.5,1,0.1)
        self.assertGreater(route['length_m'],1.5)
        self.assertTrue(all(g['data'][int(p['y']/0.1)*20+int(p['x']/0.1)]==0 for p in route['points']))
    def test_closed_route_and_unknown_rejected(self):
        g=self.grid()
        for y in range(20):g['data'][y*20+10]=100
        with self.assertRaises(ValueError):preview(g,{'x':0.5,'y':1,'frame':'map'},1.5,1,0.1)
        g['data'][10*20+15]=-1
        with self.assertRaises(ValueError):preview(g,{'x':0.5,'y':1,'frame':'map'},1.5,1,0.1)
