"""Robot CPU benchmark with synthetic frames only; no ROS subscriptions or motion."""
import shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source='import sys,types\n'
for name in ('cloud_geometry','depth_geometry'):
    code=(ROOT/('tools/'+name+'.py')).read_text(encoding='utf-8')
    source+='m=types.ModuleType('+repr(name)+');sys.modules['+repr(name)+']=m\nexec('+repr(code)+',m.__dict__)\n'
source+='''
import numpy as np,time,json,struct,math
from types import SimpleNamespace as S
from depth_geometry import depth_xyz
from cloud_geometry import transform_xyz
h=S(frame_id='camera_depth_optical_frame')
im=S(width=640,height=480,header=h,encoding='16UC1',step=1280,is_bigendian=False,data=struct.pack('<H',2000)*640*480)
info=S(width=640,height=480,header=h,p=np.array([400.,0,320,0,0,400,240,0,0,0,1,0]))
def old_projection():
 out=[];fx,fy,cx,cy=info.p[0],info.p[5],info.p[2],info.p[6]
 for v in range(0,480,8):
  for u in range(0,640,8):
   z=struct.unpack_from('<H',im.data,v*1280+u*2)[0]*.001
   out.append([(u-cx)*z/fx,(v-cy)*z/fy,z])
 return out
def old_transform(points):
 matrix=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.));translation=(0.,0.,0.)
 return [[sum(matrix[a][b]*p[b] for b in range(3))+translation[a] for a in range(3)] for p in points]
start=time.monotonic();old=old_transform(old_projection());old_ms=(time.monotonic()-start)*1000
start=time.monotonic();new=transform_xyz(depth_xyz(im,info,stride=8)[0],[0,0,0],[0,0,0,1]);new_ms=(time.monotonic()-start)*1000
error=max(abs(float(a)-float(b)) for p,q in zip(old,new) for a,b in zip(p,q))
print(json.dumps({'kind':'SYNTHETIC_CPU_ONLY','points':len(new),'old_ms':round(old_ms,2),'new_ms':round(new_ms,2),'max_error':error}),flush=True)
'''
if __name__=='__main__':
    command=['ssh','-F',str(ROOT/'config/readonly_ssh_config'),'-o','BatchMode=yes','-o','ConnectTimeout=5','cyberdog-current-nx','source /etc/mi/ros2_env.conf; python3 -u -c '+shlex.quote('import sys;exec(sys.stdin.read())')]
    result=subprocess.run(command,input=source,encoding='utf-8',capture_output=True,timeout=12)
    print(result.stdout.strip() or 'Benchmark unavailable')
