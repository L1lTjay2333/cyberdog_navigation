import ast
import json
import math
import struct
import unittest
import tempfile
import sys
import contextlib
import io
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace as Obj

from pc_probe import inspect_cloud, summarize, main


def cloud(points, big=False, double=False, width=None, padding=0):
    size = 8 if double else 4
    width = len(points) if width is None else width
    height = len(points) // width if width else 0
    data = bytearray()
    for start in range(0, len(points), width or 1):
        for point in points[start:start+width]:
            data.extend(struct.pack(('>' if big else '<') + ('ddd' if double else 'fff'), *point))
        data.extend(b'\xff' * padding)
    return Obj(width=width, height=height, point_step=size*3,
               row_step=width*size*3+padding, data=data,
               is_bigendian=big, is_dense=True,
               header=Obj(frame_id='source', stamp=Obj(sec=1, nanosec=2)),
               fields=[Obj(name=name, offset=i*size, datatype=8 if double else 7, count=1)
                       for i, name in enumerate(('x', 'y', 'z'))])


class ProbeTests(unittest.TestCase):
    def test_endianness_and_precision(self):
        for big in (True, False):
            for double in (True, False):
                result = inspect_cloud(cloud([(1, 2, 3), (4, 6, 8)], big, double))
                self.assertEqual(result['xyz_span'], [3, 4, 5])

    def test_padding(self):
        result = inspect_cloud(cloud([(1, 2, 3), (4, 5, 6)], width=1, padding=7))
        self.assertEqual(result['xyz_max'], [4, 5, 6])

    def test_nonfinite(self):
        result = inspect_cloud(cloud([(math.nan, 0, 0), (0, math.inf, 0), (1, 2, 3)]))
        self.assertEqual(result['sampled_finite_points'], 1)
        json.dumps(result, allow_nan=False)

    def test_all_invalid(self):
        result = inspect_cloud(cloud([(math.nan, 0, 0)]))
        self.assertIsNone(result['xyz_span'])

    def test_empty(self):
        self.assertEqual(inspect_cloud(cloud([]))['point_count'], 0)

    def test_missing_xyz(self):
        msg = cloud([(1, 2, 3)])
        msg.fields.pop()
        with self.assertRaises(ValueError):
            inspect_cloud(msg)

    def test_bad_layout(self):
        for attr, value in [('point_step', 0), ('row_step', 1), ('data', b'')]:
            msg = cloud([(1, 2, 3)])
            setattr(msg, attr, value)
            with self.assertRaises(ValueError):
                inspect_cloud(msg)

    def test_bad_field(self):
        for attr, value in [('datatype', 2), ('count', 2), ('offset', 99)]:
            msg = cloud([(1, 2, 3)])
            setattr(msg.fields[0], attr, value)
            with self.assertRaises(ValueError):
                inspect_cloud(msg)

    def test_sampling(self):
        result = inspect_cloud(cloud([(i, i, i) for i in range(100)]), 10)
        self.assertEqual(result['sampled_points'], 10)
        self.assertEqual(result['point_count'], 100)

    def test_summary(self):
        frames = []
        for i, stamp in enumerate((3, 3, 2)):
            frames.append(dict(stamp_ns=stamp, receive_monotonic=i,
                               sampled_finite_points=1))
        result = summarize(frames)
        self.assertEqual(result['receive_hz'], 1)
        self.assertEqual(result['duplicate_stamps'], 1)
        self.assertEqual(result['backwards_stamps'], 1)
        self.assertIsNone(summarize([])['receive_hz'])

    def test_python36_syntax_and_no_control_calls(self):
        source = (Path(__file__).resolve().parents[1] / 'tools' / 'pc_probe.py').read_text(encoding='utf-8')
        tree = ast.parse(source, feature_version=(3, 6))
        forbidden = {'create_publisher', 'create_client', 'create_service',
                     'system', 'Popen', 'run', 'call', 'check_output'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, forbidden)

    def fake_ros(self, deliver):
        state = {}
        class Node:
            def create_subscription(self, typ, topic, callback, qos):
                state['callback'] = callback
                state['qos'] = qos
                return object()
            def destroy_node(self):
                state['destroyed'] = True
        def create_node(name, **kwargs):
            state['options'] = kwargs
            return Node()
        def spin(node, timeout_sec):
            if deliver == 'interrupt':
                raise KeyboardInterrupt()
            if deliver:
                state['callback'](cloud([(1, 2, 3)]))
        modules = {'rclpy': Obj(init=lambda **kw: None, create_node=create_node,
                                spin_once=spin, shutdown=lambda: state.update(shutdown=True)),
                   'rclpy.qos': Obj(QoSProfile=lambda **kw: Obj(**kw),
                                   ReliabilityPolicy=Obj(BEST_EFFORT=1),
                                   DurabilityPolicy=Obj(VOLATILE=2)),
                   'sensor_msgs': Obj(), 'sensor_msgs.msg': Obj(PointCloud2=object)}
        return modules, state

    def test_ros_success_timeout_interrupt_and_cleanup(self):
        for delivery, status, code in [(True, 'CAPTURED', 0),
                                       (False, 'NO_MESSAGES', 2),
                                       ('interrupt', 'INTERRUPTED', 130)]:
            modules, state = self.fake_ros(delivery)
            with tempfile.TemporaryDirectory() as directory:
                output = str(Path(directory) / 'result.json')
                with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
                    actual = main(['--topic', '/cloud', '--seconds', '0.01',
                                   '--frames', '1', '--output', output])
                    self.assertEqual(actual, code)
                    with self.assertRaises(FileExistsError):
                        main(['--topic', '/cloud', '--output', output])
                result = json.loads(Path(output).read_text())
                self.assertEqual(result['status'], status)
                self.assertTrue(state['destroyed'])
                self.assertTrue(state['shutdown'])
                self.assertFalse(state['options']['enable_rosout'])
                self.assertFalse(state['options']['start_parameter_services'])


if __name__ == '__main__':
    unittest.main()
