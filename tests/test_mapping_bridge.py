import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from mapping_bridge import MappingBridge

class MappingBridgeTests(unittest.TestCase):
    def setUp(self):self.bridge=MappingBridge(Path(__file__).resolve().parents[1])
    def test_unknown_operation(self):
        with self.assertRaises(ValueError):self.bridge.start('navigate')
        self.assertFalse(self.bridge.active)
    def test_no_parallel_requests(self):
        self.bridge.active=True
        with self.assertRaises(ValueError):self.bridge.start('save')
    def test_unique_named_mapping_request(self):
        with patch('mapping_bridge.threading.Thread') as thread:
            self.bridge.start('start')
            request=thread.call_args.kwargs['args'][0]
            self.assertEqual(request['action'],'start')
            self.assertTrue(request['stop_localization'])
            self.assertTrue(request['map_name'].startswith('computer_room_'))
            self.assertTrue(self.bridge.report()['active'])
    def test_save_keeps_selected_session_name(self):
        self.bridge.name='computer_room_test'
        with patch('mapping_bridge.threading.Thread') as thread:
            self.bridge.start('save')
            self.assertEqual(thread.call_args.kwargs['args'][0]['map_name'],'computer_room_test')
            self.assertFalse(thread.call_args.kwargs['args'][0]['stop_localization'])

if __name__=='__main__':unittest.main()
