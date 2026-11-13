import ast
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import reconstruction_server as server
import reconstruction_readiness as readiness
from test_reconstruction_core import frame


class StartGateTests(unittest.TestCase):
    def old_lab(self):
        lab=server.Laboratory();lab.session='old-model'
        lab.model.add(frame(),stride=1);lab.model.add(frame(2.),stride=1)
        return lab

    def test_failed_fresh_check_preserves_previous_model_and_never_captures(self):
        lab=self.old_lab();old=lab.model
        with patch.object(server.threading,'Thread'):
            lab.start(8)
        self.assertTrue(lab.preparing);self.assertFalse(lab.active)
        self.assertIs(lab.model,old);self.assertEqual(lab.session,'old-model')
        with patch.object(server,'prepare_inputs',return_value={'ready':False,'camera_ready':False,'message':'no frames'}) as check,patch.object(lab,'capture') as capture:
            lab.prepare_job(False,8,'static')
        check.assert_called_once();capture.assert_not_called()
        self.assertIs(lab.model,old);self.assertFalse(lab.preparing);self.assertFalse(lab.running_job)

    def test_missing_pose_still_starts_raw_capture(self):
        lab=self.old_lab()
        result={'ready':False,'camera_ready':True,'task':{'status':5,'code':0}}
        with patch.object(server,'prepare_inputs',return_value=result),patch.object(lab,'capture') as capture:
            lab.prepare_job(False,8,'moving')
        capture.assert_called_once_with(8)
        self.assertTrue(lab.active)

    def test_every_start_rechecks_even_if_previous_result_was_ready(self):
        lab=self.old_lab();lab.camera={'ready':True,'message':'old success'}
        with patch.object(server,'prepare_inputs',return_value={'ready':True,'camera_ready':True,'message':'fresh'}) as check,patch.object(lab,'capture',side_effect=lambda seconds:setattr(lab,'active',False)) as capture,patch.object(server.threading,'Thread'):
            for _ in range(2):
                lab.start(8);lab.prepare_job(False,8,'static')
        self.assertEqual(check.call_count,2);self.assertEqual(capture.call_count,2)
        self.assertFalse(lab.running_job)

    def test_connect_button_never_starts_capture(self):
        lab=self.old_lab();old=lab.model
        with patch.object(server.threading,'Thread'):
            lab.connect_camera()
        with patch.object(server,'prepare_inputs',return_value={'ready':True,'message':'ready'}),patch.object(lab,'capture') as capture:
            lab.prepare_job(True,None,None)
        capture.assert_not_called();self.assertIs(lab.model,old);self.assertFalse(lab.active)

    def test_cancelled_check_does_not_start_capture(self):
        lab=self.old_lab();lab.stopped.set()
        with patch.object(server,'prepare_inputs',return_value={'ready':True}),patch.object(lab,'capture') as capture:
            lab.prepare_job(False,8,'static')
        capture.assert_not_called();self.assertFalse(lab.camera['ready'])

    def test_connect_and_start_cannot_race_a_running_job(self):
        lab=self.old_lab();lab.running_job=True
        with self.assertRaises(ValueError):lab.start(8)
        with self.assertRaises(ValueError):lab.connect_camera()

    def test_probe_failure_does_not_hide_network_denial(self):
        lab=self.old_lab()
        error=readiness.InputCheckError('SSH failed','port 22: Permission denied')
        with patch.object(server,'prepare_inputs',side_effect=error),patch.object(lab,'capture') as capture:
            lab.prepare_job(False,8,'static')
        self.assertIn('网络连接',lab.camera['message']);capture.assert_not_called()


class PreparationTests(unittest.TestCase):
    def test_plain_start_check_never_starts_mapping(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(readiness,'probe',return_value={'ready':False,'camera_ready':True,'task':{'status':101,'code':0}}),patch('mapping_bridge.MappingBridge') as bridge:
                result=readiness.prepare_inputs(Path(folder),False,threading.Event(),lambda p:None)
            bridge.assert_not_called();self.assertFalse(result['ready'])

    def test_explicit_connect_prepares_idle_mapping_then_rechecks(self):
        initial={'ready':False,'camera_ready':True,'task':{'status':101,'code':0}}
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(readiness,'probe',side_effect=[initial,{'ready':True,'camera_ready':True}]) as probe,patch('mapping_bridge.MappingBridge') as bridge:
                bridge.return_value.report.return_value={'events':[{'event':'SCANNING'}]}
                result=readiness.prepare_inputs(Path(folder),True,threading.Event(),lambda p:None)
            self.assertTrue(result['ready']);self.assertEqual(probe.call_count,2)
            request=bridge.return_value.run.call_args.args[0]
            self.assertEqual(request['action'],'start');self.assertFalse(request['stop_localization'])

    def test_other_task_is_never_stopped_by_connect(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(readiness,'probe',return_value={'ready':False,'camera_ready':True,'task':{'status':8,'code':0}}),patch('mapping_bridge.MappingBridge') as bridge:
                result=readiness.prepare_inputs(Path(folder),True,threading.Event(),lambda p:None)
            bridge.assert_not_called();self.assertFalse(result['ready'])

    def test_robot_probe_python36_and_only_known_camera_status_services(self):
        tree=ast.parse((server.ROOT/'tools/reconstruction_input_probe.py').read_text(encoding='utf-8'),feature_version=(3,6))
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertNotIn('create_publisher',calls)
        source=(server.ROOT/'tools/reconstruction_input_probe.py').read_text(encoding='utf-8')
        self.assertNotIn('SetParameters',source);self.assertNotIn('StopAlgoTask',source)


if __name__=='__main__':unittest.main()
