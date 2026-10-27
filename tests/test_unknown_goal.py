import copy
import math
import time
import unittest
from unittest.mock import patch
from unknown_goal import UnknownGoal, plan, world_cell


def grid(unknown=False):
    result = {'width':40,'height':30,'resolution':.1,'frame':'laser_odom',
              'origin':{'x':0,'y':0,'qz':0,'qw':1},'data':[0]*1200,'received_unix':time.time()}
    if unknown:
        for y in range(30):
            for x in range(25,40):result['data'][y*40+x]=-1
    return result


def snapshot(unknown=True):
    now=time.time()
    return {'connection':'connected','local_received_unix':now,'map':grid(unknown),
            'pose':{'x':.85,'y':1.55,'frame':'laser_odom','received_unix':now},
            'scan':{'received_unix':now},'mapping':{'map_name':'test_scene'}}


class UnknownGoalTests(unittest.TestCase):
    def assert_free(self,g,result,radius=.34):
        n=math.ceil(radius/g['resolution'])
        for p in result['preview']['points']:
            x,y=world_cell(g,p['x'],p['y'])
            for dy in range(-n,n+1):
                for dx in range(-n,n+1):
                    self.assertEqual(g['data'][(y+dy)*g['width']+x+dx],0)

    def test_unknown_target_plans_only_known_free_frontier(self):
        s=snapshot();target={'x':3.5,'y':1.5,'frame':'laser_odom'}
        result=plan(s['map'],s['pose'],target)
        self.assertEqual(result['status'],'FRONTIER_ROUTE')
        self.assertLess(result['waypoint']['x'],2.5)
        self.assert_free(s['map'],result)

    def test_target_outside_map_is_accepted_as_frontier(self):
        s=snapshot()
        result=plan(s['map'],s['pose'],{'x':8.,'y':1.5,'frame':'laser_odom'})
        self.assertEqual(result['status'],'FRONTIER_ROUTE');self.assert_free(s['map'],result)

    def test_new_scan_opens_complete_route(self):
        s=snapshot();session=UnknownGoal()
        result=session.set_target(s,{'x':3.3,'y':1.5})
        self.assertEqual(result['status'],'FRONTIER_ROUTE')
        s['map']['data']=[0]*1200;s['map']['received_unix']+=.01
        session.last_planned=0
        result=session.report(s)
        self.assertEqual(result['status'],'GOAL_ROUTE');self.assert_free(s['map'],result)
        self.assertFalse(result['execution_supported']);self.assertFalse(result['active'])

    def test_obstacle_detour_is_not_straight_line(self):
        s=snapshot(False);g=s['map']
        for y in range(9,21):g['data'][y*40+20]=100
        result=plan(g,s['pose'],{'x':3.2,'y':1.55,'frame':'laser_odom'})
        self.assertEqual(result['status'],'GOAL_ROUTE');self.assert_free(g,result)
        self.assertGreater(result['preview']['length_m'],3.)

    def test_complete_wall_is_no_route_not_straight_walk(self):
        s=snapshot(False)
        for y in range(30):s['map']['data'][y*40+20]=100
        result=plan(s['map'],s['pose'],{'x':3.2,'y':1.55,'frame':'laser_odom'})
        self.assertEqual(result['status'],'NO_ROUTE');self.assertIsNone(result['preview'])

    def test_goal_is_discovered_occupied(self):
        s=snapshot(False);s['map']['data'][15*40+32]=100
        result=plan(s['map'],s['pose'],{'x':3.25,'y':1.55,'frame':'laser_odom'})
        self.assertEqual(result['status'],'GOAL_BLOCKED')

    def test_start_near_wall_rejected(self):
        s=snapshot();s['pose']['x']=.05
        result=plan(s['map'],s['pose'],{'x':3.2,'y':1.5,'frame':'laser_odom'})
        self.assertEqual(result['status'],'START_BLOCKED')

    def test_distance_fixed_when_robot_moves(self):
        s=snapshot();session=UnknownGoal()
        first=session.set_target(s,{'x':3,'y':0,'relative':True})
        s['pose']['x']+=.3;s['pose']['received_unix']+=.01
        result=session.report(s)
        self.assertEqual(result['start_goal_distance_m'],3.)
        self.assertEqual(first['target'],result['target']);self.assertEqual(first['start'],result['start'])

    def test_bad_coordinate_rejected(self):
        for value in (True,None,'2',float('nan'),float('inf')):
            with self.assertRaises(ValueError):UnknownGoal().set_target(snapshot(),{'x':value,'y':0})

    def test_map_epoch_or_frame_change_pauses(self):
        for change in ('epoch','frame'):
            s=snapshot();session=UnknownGoal();session.set_target(s,{'x':3,'y':1.5})
            if change=='epoch':s['mapping']['map_name']='another_scene'
            else:s['map']['frame']=s['pose']['frame']='map'
            result=session.report(s);self.assertEqual(result['status'],'PAUSED');self.assertIsNone(result['preview'])

    def test_disconnect_and_each_stale_input_pauses(self):
        for field in ('connection','map','pose','scan','local'):
            s=snapshot();session=UnknownGoal();session.set_target(s,{'x':3,'y':1.5})
            if field=='connection':s['connection']='disconnected'
            elif field=='local':s['local_received_unix']-=10
            else:s[field]['received_unix']-=10
            result=session.report(s);self.assertEqual(result['status'],'PAUSED');self.assertIsNone(result['preview'])

    def test_offline_replay_does_not_require_live_data(self):
        s=snapshot();s['connection']='offline';s['local_received_unix']=0
        result=UnknownGoal().set_target(s,{'x':3,'y':1.5},offline=True)
        self.assertTrue(result['offline']);self.assertFalse(result['execution_supported'])

    def test_rotated_origin_routes_in_correct_frame(self):
        g=grid(False);g['origin'].update(x=10,y=20,qz=math.sin(math.pi/4),qw=math.cos(math.pi/4))
        result=plan(g,{'x':8.5,'y':20.8,'frame':'laser_odom'}, {'x':8.5,'y':23.2,'frame':'laser_odom'})
        self.assertEqual(result['status'],'GOAL_ROUTE');self.assert_free(g,result)

    def test_visited_frontiers_not_repeated(self):
        s=snapshot();target={'x':3.5,'y':1.5,'frame':'laser_odom'}
        first=plan(s['map'],s['pose'],target)
        second=plan(s['map'],s['pose'],target,visited=[first['waypoint']])
        self.assertGreaterEqual(math.hypot(second['waypoint']['x']-first['waypoint']['x'],second['waypoint']['y']-first['waypoint']['y']),.30)

    def test_clear_resets_goal_and_route(self):
        s=snapshot();session=UnknownGoal();session.set_target(s,{'x':3,'y':1.5});session.clear()
        result=session.report(s);self.assertEqual(result['status'],'IDLE');self.assertNotIn('preview',result)

    def test_preview_does_not_import_motion_transport(self):
        import unknown_goal
        self.assertNotIn('subprocess',vars(unknown_goal))
        self.assertNotIn('NavigationBridge',vars(unknown_goal))


class UnknownGoalHttpTests(unittest.TestCase):
    def test_actual_offline_endpoints_are_planning_only(self):
        import json
        import threading
        from http.server import ThreadingHTTPServer
        from urllib.request import Request,urlopen
        import map_viewer as viewer
        server=ThreadingHTTPServer(('127.0.0.1',0),viewer.Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        s=snapshot();s['connection']='offline'
        try:
            with patch.object(viewer,'OFFLINE',True),patch.object(viewer,'state',s),patch.object(viewer,'exploration',UnknownGoal()),patch.object(viewer.subprocess,'Popen') as motion,patch.object(viewer.bridge,'start') as start:
                thread.start()
                req=Request('http://127.0.0.1:%d/exploration/target'%server.server_port,
                            data=json.dumps({'x':3,'y':1.5}).encode(),
                            headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8765'})
                with urlopen(req) as response:result=json.load(response)
                self.assertTrue(result['ok']);self.assertFalse(result['exploration']['execution_supported'])
                with urlopen('http://127.0.0.1:%d/state'%server.server_port) as response:current=json.load(response)
                self.assertEqual(current['exploration']['target']['x'],3)
                motion.assert_not_called();start.assert_not_called()
        finally:server.shutdown();server.server_close();thread.join(timeout=2)
