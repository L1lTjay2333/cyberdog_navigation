import ast
import unittest
from pathlib import Path
from types import SimpleNamespace as S

class NativeMapLoadTests(unittest.TestCase):
    def fixture(self,name='scene',success=0,frame='',data=None,status=8):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text(encoding='utf-8'))
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='ensure_navigation_map')
        grid=S(header=S(frame_id=frame),info=S(width=2,height=2,resolution=.05),data=[0,100,-1,0] if data is None else data)
        calls=[];events=[]
        def call(service,*args):
            calls.append(service)
            return S(status=status,code=0) if service=='algo_task_status' else S(success=success,label=S(map_name=name,map=grid,is_outdoor=False))
        env={'operator_ready':lambda:None,'assert_active_map':lambda *a:None,'REQUEST':{'map_name':'scene'},
            'AlgoTaskStatus':S(Request=lambda:S()),'GetMapLabel':S(Request=lambda:S()),'call':call,
            'observations':{},'emit':lambda **row:events.append(row)}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'mapload','exec'),env)
        return env,grid,calls,events

    def test_map_loaded_without_topic_cache_or_motion(self):
        env,grid,calls,events=self.fixture();env['ensure_navigation_map']()
        self.assertIs(env['observations']['map'],grid)
        self.assertEqual(grid.header.frame_id,'map');self.assertEqual(grid.data,[0,100,-1,0])
        self.assertEqual(calls,['algo_task_status','get_label'])
        self.assertEqual(events[-1]['event'],'NAVIGATION_MAP_LOADED')

    def test_wrong_map_or_frame_or_incomplete_response_rejected(self):
        for kwargs in ({'name':'other'},{'success':1},{'frame':'laser_odom'},{'data':[0]}):
            env,_,_,_=self.fixture(**kwargs)
            with self.assertRaises(RuntimeError):env['ensure_navigation_map']()
            self.assertNotIn('map',env['observations'])

    def test_active_task_does_not_reload_map(self):
        env,_,calls,_=self.fixture(status=1)
        with self.assertRaises(RuntimeError):env['ensure_navigation_map']()
        self.assertEqual(calls,['algo_task_status'])
