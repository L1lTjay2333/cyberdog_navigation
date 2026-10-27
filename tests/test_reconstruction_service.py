import ast
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
import numpy as np
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
        with patch.object(server,'LAB',SimpleNamespace(start=lambda seconds,mode='static':calls.append(seconds if mode=='static' else (seconds,mode)),stop=lambda:calls.append('stop'))):
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

    def test_moving_capture_is_explicit_and_keeps_no_motion_interface(self):
        out,calls=self.request('http://127.0.0.1:8766',{'seconds':60,'mode':'moving'})
        self.assertEqual(calls,[(60,'moving')]);self.assertTrue(out[0][1]['ok'])
        lab=server.Laboratory()
        with self.assertRaises(ValueError):lab.start(60,mode='autonomous')
        self.assertFalse(lab.active)

    def test_array_and_navigation_endpoint_rejected(self):
        for data,path in [([], '/start'),({},'/navigate')]:
            out,calls=self.request('http://127.0.0.1:8766',data,path)
            self.assertFalse(calls);self.assertEqual(out[0][0],400)

    def test_export_never_fabricates_model_points(self):
        lab=server.Laboratory()
        text=lab.ply().decode('ascii')
        self.assertIn('element vertex 0\n',text)
        self.assertTrue(text.endswith('end_header\n'))

    def test_head_reference_uses_optical_forward_and_session(self):
        from test_reconstruction_core import frame
        lab=server.Laboratory();lab.session='test-head'
        lab.model.add(frame(translation=[1.,2.,3.],quaternion=[0.,float(np.sin(np.pi/4)),0.,float(np.cos(np.pi/4))]),stride=1)
        reference=lab.report()['capture_reference']
        np.testing.assert_allclose(reference['position'],[1,2,3])
        np.testing.assert_allclose(reference['forward'],[1,0,0],atol=1e-10)
        self.assertEqual(reference['session'],'test-head')
        self.assertFalse(reference['live'])

    def test_restore_model_preserves_capture_reference_without_robot(self):
        from test_reconstruction_core import frame
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder=root/'evidence/reconstruction/test-session';folder.mkdir(parents=True)
            old=server.Laboratory();old.session=folder.name
            old.model.add(frame(),stride=1);old.model.add(frame(2.),stride=1)
            (folder/'model.ply').write_bytes(old.ply())
            (folder/'summary.json').write_text(json.dumps(old.report(False)))
            (folder/'frame_01.json').write_text(json.dumps(frame()))
            lab=server.Laboratory()
            with patch.object(server,'ROOT',root),patch.object(server.subprocess,'Popen') as process:
                self.assertTrue(lab.restore_latest())
            process.assert_not_called();self.assertFalse(lab.active)
            self.assertEqual(lab.report()['stable_points'],4)
            self.assertEqual(lab.report()['capture_reference']['session'],folder.name)

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
