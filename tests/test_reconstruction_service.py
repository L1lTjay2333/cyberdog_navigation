import ast
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import patch
import reconstruction_server as server


class ServiceTests(unittest.TestCase):
    def request(self, origin, data, path='/start'):
        handler=object.__new__(server.Handler)
        raw=json.dumps(data).encode()
        handler.headers={'Origin':origin,'Content-Type':'application/json','Content-Length':str(len(raw))}
        handler.path=path;handler.rfile=io.BytesIO(raw);out=[]
        handler.send=lambda data,kind='json',status=200:out.append((status,json.loads(data)))
        calls=[]
        with patch.object(server,'LAB',SimpleNamespace(start=lambda seconds:calls.append(seconds),stop=lambda:calls.append('stop'))):
            handler.do_POST()
        return out,calls

    def test_untrusted_site_cannot_start_capture(self):
        out,calls=self.request('https://untrusted.invalid',{})
        self.assertEqual(calls,[])
        self.assertEqual(out[0][0],400)

    def test_local_capture_and_stop_only(self):
        out,calls=self.request('http://127.0.0.1:8766',{'seconds':12})
        self.assertEqual(calls,[12]);self.assertTrue(out[0][1]['ok'])
        out,calls=self.request('http://127.0.0.1:8766',{},'/stop')
        self.assertEqual(calls,['stop'])

    def test_array_and_navigation_endpoint_rejected(self):
        for data,path in [([], '/start'),({},'/navigate')]:
            out,calls=self.request('http://127.0.0.1:8766',data,path)
            self.assertFalse(calls);self.assertEqual(out[0][0],400)

    def test_export_never_fabricates_model_points(self):
        lab=server.Laboratory()
        text=lab.ply().decode('ascii')
        self.assertIn('element vertex 0\n',text)
        self.assertTrue(text.endswith('end_header\n'))

    def test_early_stop_does_not_open_a_connection_or_report_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            lab=server.Laboratory();lab.session='cancelled_test';lab.active=True;lab.stopped.set()
            with patch.object(server,'ROOT',Path(directory)),patch.object(server.subprocess,'Popen') as process:
                lab.capture(1)
            process.assert_not_called()
            self.assertIsNone(lab.error)
            self.assertFalse(lab.active)

    def test_output_setup_failure_does_not_leave_session_busy(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'evidence').write_text('not a directory')
            lab=server.Laboratory();lab.session='failed_setup';lab.active=True
            with patch.object(server,'ROOT',root),patch.object(server.subprocess,'Popen') as process:
                lab.capture(1)
            process.assert_not_called()
            self.assertFalse(lab.active)
            self.assertIsNotNone(lab.error)

    def test_robot_capture_compatible_and_has_no_control_calls(self):
        p=Path(server.ROOT/'tools/reconstruction_capture.py')
        tree=ast.parse(p.read_text(encoding='utf-8'),feature_version=(3,6))
        called={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse({'create_publisher','create_client'}&called)
        self.assertIn('lookup_transform',called)


if __name__=='__main__':unittest.main()
