import ast
import math
from pathlib import Path
import types
import unittest
from unittest.mock import Mock

class LocalizationOnlyTests(unittest.TestCase):
    def test_actual_robot_main_exits_before_any_navigation_or_plan(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py').read_text(encoding='utf-8'))
        classes=[n for n in tree.body if isinstance(n,ast.ClassDef)]
        main=next(n for n in tree.body if isinstance(n,ast.Try))
        item=types.SimpleNamespace;events=[];ticks=iter([0,0,5]);planner=Mock();transport=Mock()
        transform=item(header=item(stamp=item(sec=100,nanosec=0)),transform=item(translation=item(x=1,y=2,z=0),rotation=item(x=0,y=0,z=0,w=1)))
        env={'REQUEST':{'action':'localize'},'refresh_localization':lambda:50,'operator_ready':lambda:None,'heartbeat':{'ready':True,'operator_live':True,'stop':False},
             'wait_for_navigation_inputs':lambda *_:None,
             'wait_for_stable_localization':lambda:transform,
             'time':item(time=lambda:100,monotonic=lambda:next(ticks)),
             'buffer':item(lookup_transform=lambda *_:transform),
             'rclpy':item(time=item(Time=lambda:None),shutdown=lambda:None),
             'math':math,'emit':lambda **e:events.append(e),'own_uuid':None,'client':None,
             'node':item(destroy_node=lambda:None),'listener_done':item(set=lambda:None),'heartbeat_thread':item(join=lambda **_:None,is_alive=lambda:False),
             'os':item(close=lambda _:None),'heartbeat_descriptor':0,
             'replan_from_current_position':planner,'ActionClient':transport}
        exec(compile(ast.Module(body=classes+[main],type_ignores=[]),'localization_only','exec'),env)
        self.assertEqual(events,[{'event':'POSITION_REFRESHED','x':1,'y':2,'yaw':0.,'frame':'map'}])
        planner.assert_not_called();transport.assert_not_called()

if __name__=='__main__':unittest.main()
