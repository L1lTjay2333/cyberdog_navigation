import base64,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from native_photo_capture import jpeg_size,save
from reconstruction_server import Laboratory


class NativePhotoTests(unittest.TestCase):
    def test_actual_native_jpeg_dimensions(self):
        p=Path('evidence/highres_camera_20261101/native_20261101_162819.jpg')
        self.assertEqual(jpeg_size(p.read_bytes()),(4208,3120))

    def test_save_preserves_original_and_rejects_bad_payload(self):
        raw=Path('evidence/highres_camera_20261101/native_20261101_162819.jpg').read_bytes()
        with tempfile.TemporaryDirectory() as d:
            row=dict(kind='native_photo',data=base64.b64encode(raw).decode())
            r=save(Path(d),None,row)
            self.assertEqual(Path(r['file']).read_bytes(),raw)
            self.assertEqual(r['session'],'unassigned')
            with self.assertRaises(ValueError):jpeg_size(raw[:30])

    def test_stationary_required_and_cooldown_prevents_repeat(self):
        lab=Laboratory()
        with self.assertRaises(ValueError):lab.take_native_photo(False)
        with patch('reconstruction_server.threading.Thread') as thread:
            lab.take_native_photo(True)
            thread.return_value.start.assert_called_once()
            with self.assertRaises(ValueError):lab.take_native_photo(True)
