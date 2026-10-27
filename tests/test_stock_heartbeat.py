"""Offline regression: remote heartbeat reader must finish without stdin locks."""
import ast
import collections
import json
from pathlib import Path
import threading
import time
import types
import unittest

class HeartbeatTests(unittest.TestCase):
    def setup_reader(self,chunks):
        source=Path(__file__).resolve().parents[1]/'tools/stock_navigation_session.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='listen')
        queue=collections.deque(chunks)
        done=threading.Event();heartbeat={'stop':False,'last':0}
        def ready(*args):
            if queue:return ([0],[],[])
            done.wait(.01);return ([],[],[])
        env={'sys':types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda:0)),
             'select':types.SimpleNamespace(select=ready),
             'os':types.SimpleNamespace(read=lambda *_:queue.popleft()),
             'json':json,'time':time,'listener_done':done,'heartbeat':heartbeat}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'heartbeat','exec'),env)
        thread=threading.Thread(target=env['listen']);thread.start()
        return done,heartbeat,thread
    def test_idle_reader_exits_cleanly(self):
        done,_,thread=self.setup_reader([])
        done.set();thread.join(1)
        self.assertFalse(thread.is_alive())
    def test_fragmented_cancel_and_eof(self):
        _,heartbeat,thread=self.setup_reader([b'{"heartbeat":tr',b'ue}\n{"cancel":true}\n',b''])
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertTrue(heartbeat['stop'])
        self.assertGreater(heartbeat['last'],0)

if __name__=='__main__':unittest.main()
