"""Local endpoint tests; mock every robot transport and motion start."""
import json
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request,urlopen
from unittest.mock import patch
import map_viewer as viewer
from localization_validity import LocalizationValidity

class NavigationHttpTests(unittest.TestCase):
    def setUp(self):
        now=time.time()
        self.snapshot={'connection':'connected','local_received_unix':now,'emitted_unix':now,
            'task':{'status':8,'received_unix':now},'pose':{'x':.5,'y':.5,'yaw':0,'frame':'map','received_unix':now},
            'battery':{'percent':50,'received_unix':now,'wired_charging':False},
            'scan':{'received_unix':now,'finite_ranges':100},
            'map':{'frame':'map','width':5,'height':5,'resolution':1,'origin':{'x':0,'y':0,'qz':0,'qw':1},'data':[0]*25}}
        self.target={'x':1.5,'y':.5}
        route={'frame':'map','kind':'stock_nav2_preview','length_m':1,'points':[{'x':.5,'y':.5},self.target]}
        self.validity=LocalizationValidity();self.validity.required=False
        self.patches=[patch.object(viewer,'OFFLINE',False),patch.object(viewer,'state',self.snapshot),
            patch.object(viewer,'localization_validity',self.validity),
            patch.object(viewer,'selection',None),patch.object(viewer.manual,'started',False),
            patch.object(viewer.mapping,'report',return_value={'active':False}),
            patch.object(viewer.explorer,'report',return_value={'active':False}),
            patch.object(viewer.bridge,'report',return_value={'active':False,'enabled':True,'max_goal_distance_m':2.8})]
        for p in self.patches:p.start()
        self.preview_patch=patch.object(viewer,'stock_preview',return_value=route);self.preview=self.preview_patch.start()
        self.start_patch=patch.object(viewer.bridge,'start');self.start=self.start_patch.start()
        self.localize_patch=patch.object(viewer.bridge,'start_localization');self.localize=self.localize_patch.start()
        self.server=ThreadingHTTPServer(('127.0.0.1',0),viewer.Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
        self.start_patch.stop();self.preview_patch.stop()
        self.localize_patch.stop()
        for p in reversed(self.patches):p.stop()
    def post(self,path,target=None):
        req=Request('http://127.0.0.1:%d%s'%(self.server.server_port,path),
            data=json.dumps(target or self.target).encode(),headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8765'})
        try:
            with urlopen(req,timeout=3) as r:return r.status,json.load(r)
        except HTTPError as e:return e.code,json.load(e)
    def test_recent_preview_is_reused_without_second_ssh_planning(self):
        self.assertEqual(self.post('/goal')[0],200);self.assertEqual(self.post('/navigate')[0],200)
        self.assertEqual(self.preview.call_count,1);self.start.assert_called_once()
    def test_unchanged_map_is_omitted_without_mutating_shared_snapshot(self):
        self.snapshot['display_map_revision']='stream:1'
        with patch.object(viewer.explorer,'report',return_value={'active':False,'deployed':False}),patch.object(viewer.bridge,'report',return_value={'active':False,'enabled':True,'events':[]}):
            with urlopen('http://127.0.0.1:%d/state?map_revision=stream:1'%self.server.server_port) as r:
                result=json.load(r)
            self.assertNotIn('data',result['map'])
            self.assertEqual(len(self.snapshot['map']['data']),25)
            with urlopen('http://127.0.0.1:%d/state?map_revision=old'%self.server.server_port) as r:
                self.assertEqual(len(json.load(r)['map']['data']),25)
    def test_far_target_fails_before_ssh(self):
        code,result=self.post('/goal',{'x':4,'y':.5})
        self.assertEqual(code,400);self.assertIn('2.8',result['message'])
        self.preview.assert_not_called();self.start.assert_not_called()
    def test_charging_transition_invalidates_old_position(self):
        self.snapshot['battery']['wired_charging']=True
        self.assertEqual(self.post('/goal')[0],400)
        code,result=self.post('/navigate');self.assertEqual(code,400);self.assertIn('充电',result['message'])
        self.start.assert_not_called()
    def test_relocation_blocks_goal_and_regular_check_starts_new_localization(self):
        self.validity.invalidate('已搬动，需要重新定位')
        self.assertEqual(self.post('/navigate')[0],400)
        self.start.assert_not_called();self.preview.assert_not_called()
        code,result=self.post('/relocalize',{'force':False})
        self.assertEqual(code,200);self.assertFalse(result['reused'])
        self.localize.assert_called_once()
    def test_expired_preview_is_recomputed(self):
        self.post('/goal');viewer.selection['prepared_unix']-=31
        self.assertEqual(self.post('/navigate')[0],200);self.assertEqual(self.preview.call_count,2)

    def test_unchanged_scene_preview_remains_reusable_after_six_seconds(self):
        self.post('/goal');viewer.selection['prepared_unix']-=6
        self.assertEqual(self.post('/navigate')[0],200);self.assertEqual(self.preview.call_count,1)

    def test_changed_map_requires_a_new_preview(self):
        self.post('/goal');self.snapshot['map']['received_unix']=time.time()
        self.assertEqual(self.post('/navigate')[0],200);self.assertEqual(self.preview.call_count,2)

    def test_moved_robot_requires_a_new_preview(self):
        self.post('/goal');self.snapshot['pose']['x']+=.2
        self.assertEqual(self.post('/navigate')[0],200);self.assertEqual(self.preview.call_count,2)

    def test_low_percentage_does_not_block_web_navigation(self):
        self.snapshot['battery']['percent']=9
        self.assertEqual(self.post('/goal')[0],200)
        self.assertEqual(self.post('/navigate')[0],200)
        self.start.assert_called_once()

    def test_valid_localization_is_reused_and_target_preserved(self):
        self.post('/goal');selected=viewer.selection
        code,result=self.post('/relocalize',{'force':False})
        self.assertEqual(code,200);self.assertTrue(result['reused'])
        self.assertIs(viewer.selection,selected)
        self.localize.assert_not_called()

    def test_force_localization_starts_once_and_keeps_target(self):
        self.post('/goal')
        selected=dict(viewer.selection)
        code,result=self.post('/relocalize',{'force':True})
        self.assertEqual(code,200);self.assertFalse(result['reused'])
        self.localize.assert_called_once();self.assertEqual(viewer.selection,selected)

    def test_stale_position_cannot_be_reused(self):
        self.snapshot['pose']['received_unix']-=5
        code,result=self.post('/relocalize',{'force':False})
        self.assertEqual(code,200);self.assertFalse(result['reused'])
        self.localize.assert_called_once()

    def test_manual_stale_reset_requires_robot_confirmation_and_invalidates_pose(self):
        self.snapshot['task']['status']=1
        with patch.object(viewer,'reset_stale_navigation',return_value={'ok':True,'message':'已清理'}) as reset:
            code,result=self.post('/task/reset_stale',{'stopped':True})
        self.assertEqual(code,200);self.assertTrue(result['ok'])
        reset.assert_called_once();self.assertTrue(self.validity.required)
        self.start.assert_not_called();self.localize.assert_not_called()
