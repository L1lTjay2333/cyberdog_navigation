"""No SSH: verify successful saves select the next localization map."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from mapping_bridge import MappingBridge

class SavedMapConfigTests(unittest.TestCase):
    def run_save(self,returncode,events):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for directory in ('tools','config','evidence'):(root/directory).mkdir()
            (root/'tools/mapping_session.py').write_text('pass',encoding='utf-8')
            for name in ('camera_handoff.py','map_inventory.py'):
                (root/'tools'/name).write_text('pass',encoding='utf-8')
            old={'computer_map_name':'old_room','motion_enabled':False,'manual_motion_enabled':False}
            (root/'config/scope.json').write_text(json.dumps(old),encoding='utf-8')
            result=SimpleNamespace(returncode=returncode,stdout='\n'.join(json.dumps(e) for e in events))
            bridge=MappingBridge(root)
            with patch('mapping_bridge.subprocess.run',return_value=result):
                bridge.run({'action':'save','map_name':'new_room'})
            return json.loads((root/'config/scope.json').read_text()),bridge.report()

    def test_confirmed_save_updates_map_without_unlocking_motion(self):
        scope,report=self.run_save(0,[{'event':'SAVED'}])
        self.assertEqual(scope['computer_map_name'],'new_room')
        self.assertIs(scope['motion_enabled'],False)
        self.assertIs(scope['manual_motion_enabled'],False)
        self.assertFalse(report['active'])

    def test_acknowledgement_only_does_not_select_unsaved_map(self):
        scope,_=self.run_save(0,[{'event':'ACK','accepted':True}])
        self.assertEqual(scope['computer_map_name'],'old_room')

    def test_failed_process_keeps_old_map(self):
        scope,_=self.run_save(1,[{'event':'SAVED'}])
        self.assertEqual(scope['computer_map_name'],'old_room')
