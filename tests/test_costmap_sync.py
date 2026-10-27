import ast
import unittest
from pathlib import Path
from types import SimpleNamespace as N

class CostmapTests(unittest.TestCase):
    def test_exact_geometry_required(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'tools/stock_plan_once.py').read_text(encoding='utf-8'))
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='matching_geometry')
        env={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'geometry','exec'),env)
        def grid(w):return N(header=N(frame_id='map'),info=N(width=w,height=235,resolution=.05,
            origin=N(position=N(x=0,y=0),orientation=N(z=0,w=1))))
        self.assertTrue(env['matching_geometry'](grid(189),grid(189)))
        self.assertFalse(env['matching_geometry'](grid(189),grid(203)))
        a,b=grid(189),grid(189);b.info.origin.position.x=1
        self.assertFalse(env['matching_geometry'](a,b))
