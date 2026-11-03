import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace as S
from cloud_geometry import transform_xyz

class TransformTest(unittest.TestCase):
 def test_timestamped_pose_and_camera_composition(self):
  tree=ast.parse(Path('tools/map_stream.py').read_text(encoding='utf-8'))
  fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='display_transform')
  def tf(x,y,z,q):return S(translation=S(x=x,y=y,z=z),rotation=S(x=q[0],y=q[1],z=q[2],w=q[3]))
  def out():return S(header=S(),transform=tf(0,0,0,[0,0,0,1]))
  calls=[]
  class Buffer:
   def __init__(self,value):self.value=value
   def lookup_transform(self,target,source,stamp):calls.append((target,source,stamp));return S(transform=self.value)
  stamp=object();angle=math.sqrt(.5)
  env={'state':{'task':{'status':5}},'mapping_pose_buffer':Buffer(tf(1,2,3,[0,0,angle,angle])),
       'tf_buffer':Buffer(tf(.2,0,.1,[0,0,0,1])),'TransformStamped':out,'transform_xyz':transform_xyz,
       'rclpy':S(time=S(Time=S(from_msg=lambda s:s)))}
  exec(compile(ast.Module(body=[fn],type_ignores=[]),'display_transform','exec'),env)
  t=env['display_transform']('laser_odom','camera',stamp)
  for actual,want in zip([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],[1,2.2,3.1]):self.assertAlmostEqual(actual,want)
  self.assertEqual(calls,[('laser_odom','base_link',stamp),('base_link','camera',stamp)])
  self.assertAlmostEqual(t.transform.rotation.z,angle)
  # Missing timestamped pose must fail rather than substituting the latest pose.
  env['mapping_pose_buffer'].lookup_transform=lambda *a:(_ for _ in ()).throw(ValueError('not ready'))
  with self.assertRaises(ValueError):env['display_transform']('laser_odom','camera',stamp)
