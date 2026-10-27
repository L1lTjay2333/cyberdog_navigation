import ast
import types
import unittest
from pathlib import Path


class EfficiencyTests(unittest.TestCase):
    def setup_profile(self,task=8,configured=False,fail_configure=False,initial_state=2):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text(encoding='utf-8'))
        nodes=[n for n in tree.body if (isinstance(n,ast.FunctionDef) and n.name=='apply_navigation_efficiency_profile') or
               (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='NAVIGATION_EFFICIENCY_PARAMETERS' for t in n.targets))]
        item=types.SimpleNamespace;events=[];calls=[];storage={};state=[initial_state]
        class FakeParameter:
            def __init__(self,name,value=None):self.name=name;self.value=value
            def to_parameter_msg(self):
                value=self.value
                kind=1 if type(value)==bool else 2 if type(value)==int else 3 if type(value)==float else 0
                return item(name=self.name,value=item(type=kind,double_value=value if kind==3 else 0.,integer_value=value if kind==2 else 0,bool_value=value if kind==1 else False))
        env={'operator_ready':lambda:None,'Parameter':FakeParameter,'emit':lambda **v:events.append(v)}
        for name in ('GetParameters','SetParametersAtomically','GetState','AlgoTaskStatus'):
            env[name]=item(Request=lambda:item())
        env['ChangeState']=item(Request=lambda:item(transition=item(id=0)))
        def call(name,kind,request):
            calls.append(name)
            if name=='algo_task_status':return item(status=task,code=0)
            if name.endswith('/get_state'):return item(current_state=item(id=state[0]))
            if name.endswith('/get_parameters'):return item(values=[storage[n] for n in request.names])
            if name.endswith('/set_parameters_atomically'):
                storage.update({p.name:p.value for p in request.parameters});return item(result=item(successful=True,reason=''))
            if name.endswith('/change_state'):
                if request.transition.id==1 and fail_configure and not any(e['event']=='NAVIGATION_PROFILE_ROLLED_BACK' for e in events):
                    # Only the first configure fails; rollback configure succeeds.
                    nonlocal_fail[0]+=1
                    if nonlocal_fail[0]==1:return item(success=False)
                state[0]={1:2,2:1,4:2}[request.transition.id];return item(success=True)
            raise AssertionError(name)
        nonlocal_fail=[0];env['call']=call
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'profile','exec'),env)
        for name,value in env['NAVIGATION_EFFICIENCY_PARAMETERS'].items():
            storage[name]=FakeParameter(name,value if configured else (.4 if type(value)==float else 10 if type(value)==int else True)).to_parameter_msg().value
        return env,events,calls,storage,state

    def test_reload_applies_and_leaves_controller_inactive(self):
        env,events,calls,storage,state=self.setup_profile();env['apply_navigation_efficiency_profile']()
        self.assertEqual(state[0],2)
        self.assertEqual(events[-1]['event'],'NAVIGATION_PROFILE_APPLIED')
        self.assertEqual(storage['FollowPath.vtheta_samples'].integer_value,41)
        self.assertEqual(storage['FollowPath.max_vel_x'].double_value,.65)
        self.assertEqual(storage['FollowPath.max_speed_xy'].double_value,.65)
        self.assertEqual(storage['FollowPath.GoalAlign.scale'].double_value,12.)
        self.assertEqual(storage['FollowPath.PathAlign.scale'].double_value,16.)
        self.assertEqual(storage['FollowPath.GoalDist.scale'].double_value,36.)
        self.assertEqual(storage['FollowPath.acc_lim_x'].double_value,1.3)
        self.assertEqual(storage['FollowPath.decel_lim_x'].double_value,-1.)
        self.assertEqual(storage['FollowPath.PathAlign.forward_point_distance'].double_value,.35)
        self.assertEqual(storage['FollowPath.GoalAlign.forward_point_distance'].double_value,.35)
        self.assertEqual(storage['FollowPath.acc_lim_theta'].double_value,1.)
        self.assertEqual(storage['FollowPath.decel_lim_theta'].double_value,-1.)
        self.assertTrue(any(n.endswith('/change_state') for n in calls))

    def test_already_configured_does_not_reload(self):
        env,events,calls,_,_=self.setup_profile(configured=True);env['apply_navigation_efficiency_profile']()
        self.assertEqual(events[-1]['event'],'NAVIGATION_PROFILE_REUSED')
        self.assertFalse(any(n.endswith('/change_state') for n in calls))

    def test_fresh_controller_configured_before_reading_plugin_parameters(self):
        env,events,calls,_,state=self.setup_profile(initial_state=1)
        env['apply_navigation_efficiency_profile']()
        self.assertLess(calls.index('controller_server_ab/change_state'),calls.index('controller_server_ab/get_parameters'))
        self.assertEqual(state[0],2)
        self.assertTrue(any(e['event']=='NAVIGATION_CONTROLLER_PREPARED' for e in events))

    def test_active_task_never_written(self):
        env,_,calls,_,_=self.setup_profile(task=1)
        with self.assertRaises(RuntimeError):env['apply_navigation_efficiency_profile']()
        self.assertEqual(calls,['algo_task_status'])

    def test_failed_reload_restores_original(self):
        env,events,_,storage,state=self.setup_profile(fail_configure=True)
        with self.assertRaises(RuntimeError):env['apply_navigation_efficiency_profile']()
        self.assertEqual(storage['FollowPath.max_vel_x'].double_value,.4)
        self.assertEqual(events[-1]['event'],'NAVIGATION_PROFILE_ROLLED_BACK')
        self.assertEqual(state[0],2)

    def test_zero_yaw_sample_and_obstacle_critic_unchanged(self):
        env,_,_,_,_=self.setup_profile();p=env['NAVIGATION_EFFICIENCY_PARAMETERS']
        self.assertEqual(p['FollowPath.vtheta_samples']%2,1)
        self.assertEqual(p['FollowPath.min_speed_theta'],0.)
        self.assertNotIn('FollowPath.BaseObstacle.scale',p)
        self.assertNotIn('FollowPath.critics',p)
        self.assertEqual(p['FollowPath.max_vel_theta'],.8)
        self.assertEqual(p['FollowPath.decel_lim_x'],-1.)
