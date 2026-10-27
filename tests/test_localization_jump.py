import ast
import math
from pathlib import Path
import unittest

class LocalizationJumpTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='check_localization_jump')
        env={'math':math}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'jump_gate','exec'),env)
        self.check=env['check_localization_jump']
        self.pose={'x':0.,'y':0.,'yaw':0.,'frame':'map'}

    def test_small_change_allowed(self):
        self.check(self.pose,.03,.02,.02)

    def test_four_meter_jump_rejected(self):
        with self.assertRaises(RuntimeError):self.check(self.pose,0.,4.,0.)

    def test_half_turn_rejected(self):
        with self.assertRaises(RuntimeError):self.check(self.pose,0.,0.,math.pi)

    def test_wraparound_allowed(self):
        self.pose['yaw']=math.pi-.01
        self.check(self.pose,0.,0.,-math.pi+.01)

    def test_bad_reference_rejected(self):
        self.pose['x']=float('nan')
        with self.assertRaises(RuntimeError):self.check(self.pose,0.,0.,0.)
