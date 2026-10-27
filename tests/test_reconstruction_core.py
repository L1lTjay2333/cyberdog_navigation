import base64
import math
import struct
import unittest
import zlib
import numpy as np

try:
    from reconstruction_core import decode_depth_frame, StaticModel, FrameQueue
except ImportError:
    decode_depth_frame = StaticModel = FrameQueue = None


def frame(stamp=1., values=(1000,1000,1000,1000), **extra):
    raw=struct.pack('<HHHH',*values)
    data={'stamp':stamp,'frame':'vodom','source_frame':'camera_depth_optical_frame',
          'width':2,'height':2,'step':4,'encoding':'16UC1','bigendian':False,
          'intrinsics':[100.,100.,.5,.5], 'translation':[0.,0.,0.], 'quaternion':[0.,0.,0.,1.],
          'depth_zlib_b64':base64.b64encode(zlib.compress(raw)).decode('ascii')}
    data.update(extra)
    return data


class ReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(decode_depth_frame,'Reconstruction core has not been implemented')

    def test_projection_and_pose(self):
        d=frame(translation=[1.,2.,3.])
        pts=decode_depth_frame(d,stride=1)
        np.testing.assert_allclose(pts[0],[.995,1.995,4.])

    def test_bigendian_and_padded_float_depth(self):
        raw=struct.pack('>ff',1.,float('nan'))+b'PAD!'+struct.pack('>ff',0.,2.)+b'PAD!'
        d=frame(encoding='32FC1',bigendian=True,step=12,
                depth_zlib_b64=base64.b64encode(zlib.compress(raw)).decode('ascii'))
        pts=decode_depth_frame(d,stride=1)
        self.assertEqual(len(pts),2)
        np.testing.assert_allclose(pts[:,2],[1.,2.])

    def test_rotation_uses_full_quaternion(self):
        pts=decode_depth_frame(frame(quaternion=[0.,math.sin(math.pi/4),0.,math.cos(math.pi/4)]),stride=1)
        np.testing.assert_allclose(pts[0],[1.,-.005,.005],atol=1e-10)

    def test_malformed_and_bomb_payloads_rejected(self):
        for d in [frame(step=2),frame(encoding='mono16'),frame(quaternion=[0.,0.,0.,0.]),
                  frame(intrinsics=[0.,100.,0.,0.]),frame(stamp=float('nan')),
                  frame(intrinsics=[1e-320,100.,.5,.5]),
                  frame(tf_stamp=2.),
                  frame(depth_zlib_b64=base64.b64encode(zlib.compress(b'X'*100000)).decode('ascii'))]:
            with self.subTest(d=d['encoding']):
                with self.assertRaises(ValueError):decode_depth_frame(d)

    def test_duplicate_frames_cannot_increase_support(self):
        model=StaticModel(voxel=.02)
        model.add(frame(),stride=1)
        self.assertEqual(len(model.points()),0)
        self.assertFalse(model.add(frame(),stride=1))
        model.add(frame(2.),stride=1)
        self.assertEqual(len(model.points()),4)

    def test_pose_motion_and_frame_change_freeze_before_mutation(self):
        for changed in [frame(2.,translation=[.1,0,0]),frame(2.,frame='map'),frame(2.,source_frame='other_camera'),
                        frame(2.,quaternion=[0.,0.,math.sin(.1),math.cos(.1)])]:
            model=StaticModel();model.add(frame(),stride=1)
            before=model.stats().copy()
            with self.assertRaises(ValueError):model.add(changed,stride=1)
            self.assertEqual(model.stats()['accepted_frames'],before['accepted_frames'])

    def test_centroid_and_distinct_frame_support(self):
        model=StaticModel(voxel=.1)
        model.add(frame(values=(1001,1001,1001,1001)),stride=1)
        model.add(frame(2.,values=(1003,1003,1003,1003)),stride=1)
        self.assertTrue(all(abs(p[2]-1.002)<1e-10 for p in model.points()))
        self.assertEqual(model.stats()['accepted_frames'],2)

    def test_capacity_never_silently_discards_old_surface(self):
        model=StaticModel(max_voxels=1)
        with self.assertRaises(ValueError):model.add(frame(),stride=1)
        self.assertEqual(model.stats()['voxels'],0)

    def test_unrepresentable_voxel_coordinates_do_not_mutate_model(self):
        for model,data in [(StaticModel(voxel=1e-320),frame()),
                           (StaticModel(),frame(translation=[1e30,0,0]))]:
            with self.assertRaises(ValueError):model.add(data,stride=1)
            self.assertEqual(model.stats()['voxels'],0)
            self.assertEqual(model.stats()['accepted_frames'],0)

    def test_queue_keeps_original_stamp_and_retries_until_ready(self):
        queue=FrameQueue(capacity=2,max_age=2.,wait=.1)
        queue.offer({'stamp':1.},now=0.)
        self.assertEqual(queue.ready(lambda f:None,now=.05),[])
        self.assertEqual(queue.ready(lambda f:None,now=.5),[])
        self.assertEqual(queue.ready(lambda f:{'matched':f['stamp']},now=.8),[({'stamp':1.},{'matched':1.})])

    def test_queue_capacity_and_expiration(self):
        queue=FrameQueue(capacity=2,max_age=2.,wait=.1)
        for i in range(3):queue.offer({'stamp':float(i)},now=0.)
        self.assertEqual(queue.dropped,1)
        queue.ready(lambda f:None,now=2.1)
        self.assertEqual(queue.expired,2)


if __name__=='__main__':unittest.main()
