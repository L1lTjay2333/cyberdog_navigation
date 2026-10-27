import ast
from pathlib import Path
from types import SimpleNamespace as Item
import unittest

class PreviewLifecycleTests(unittest.TestCase):
    def run_prepare(self,state=1,status=8):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_plan_once.py').read_text(encoding='utf-8'))
        func=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare_planner')
        calls=[];states=iter([state,2,3])
        def call(name,kind,request):
            calls.append((name,getattr(getattr(request,'transition',None),'id',None)))
            if name=='algo_task_status':return Item(status=status,code=0)
            if name.endswith('get_state'):return Item(current_state=Item(id=next(states)))
            return Item(success=True)
        env={'service_call':call,'AlgoTaskStatus':Item(Request=lambda:Item()),'GetState':Item(Request=lambda:Item()),'ChangeState':Item(Request=lambda:Item(transition=Item(id=0)))}
        exec(compile(ast.Module(body=[func],type_ignores=[]),'preview_lifecycle','exec'),env)
        env['ensure_current_costmap']=lambda:None
        env['prepare_planner']()
        return calls

    def test_unconfigured_planner_configured_and_activated(self):
        calls=self.run_prepare()
        self.assertEqual([t for n,t in calls if n.endswith('change_state')],[1,3])
        self.assertTrue(all(n=='algo_task_status' or n.startswith('planner_server_ab/') for n,_ in calls))

    def test_active_planner_untouched(self):
        self.assertFalse(any(n.endswith('change_state') for n,_ in self.run_prepare(state=3)))

    def test_no_localization_rejected(self):
        with self.assertRaises(ValueError):self.run_prepare(status=6)
