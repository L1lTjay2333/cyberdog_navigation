"""Audit room floor orientation and re-integrate original depth for offline comparison."""
import json,sys
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation,Slerp
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from reconstruction_core import decode_depth_frame
root=Path(__file__).resolve().parents[1]
out=root/'evidence/room_analysis_20261105'
poses=np.loadtxt(out/'stock_pose.txt')
interpolate=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:8]))
records=[]
with (root/'evidence/live_cloud_maps/session_1793867659_b7675ab9/frames.jsonl').open() as f:
 for line in f:records.append(json.loads(line))
audit=[];cells={}
for i,r in enumerate(records):
 p=np.array(r['points_xyz']);d=r['depth']
 if i%5==0:
  rest=p.copy();floor=None
  for attempt in range(4):
   if len(rest)<50:break
   cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(rest))
   plane,ids=cloud.segment_plane(.025,3,150)
   n=np.array(plane[:3])
   if abs(n[2])>.85:
    floor=dict(normal=n.tolist(),tilt_deg=float(np.degrees(np.arccos(abs(n[2])))),
               height_at_camera_xy=float(-(np.dot(n[:2],d['translation'][:2])+plane[3])/n[2]),support=len(ids))
    break
   rest=np.delete(rest,ids,axis=0)
  item=dict(frame=i,stamp=r['stamp'],horizontal_plane=floor)
  t=d['stamp']
  if poses[0,0]<=t<=poses[-1,0]:
   base=interpolate([t]).as_matrix()[0]
   camera=Rotation.from_quat(d['quaternion']).as_matrix()
   item['base_to_camera_rotation']=(Rotation.from_matrix(base.T@camera).as_rotvec()).tolist()
   xyz=np.array([np.interp(t,poses[:,0],poses[:,j]) for j in (1,2,3)])
   item['base_to_camera_translation']=(base.T@(np.array(d['translation'])-xyz)).tolist()
  audit.append(item)
 # Consensus without plane snapping or invented wall geometry.
 _,idx=np.unique(np.floor(p/.05),axis=0,return_index=True)
 for v in p[idx]:
  k=tuple(np.floor(v/.05).astype(int))
  if k not in cells:cells[k]=[v.copy(),1]
  else:cells[k][0]+=v;cells[k][1]+=1
points=np.array([v/n for v,n in cells.values() if n>=3])
cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
cloud,_=cloud.remove_statistical_outlier(20,2)
o3d.io.write_point_cloud(str(out/'consensus_cleaned.ply'),cloud)
planes=[x['horizontal_plane'] for x in audit if x['horizontal_plane']]
heights=[x['height_at_camera_xy'] for x in planes]
tilts=[x['tilt_deg'] for x in planes]
offsets=np.array([x['base_to_camera_translation'] for x in audit if 'base_to_camera_translation' in x])
report=dict(sampled_frames=len(audit),horizontal_planes=len(planes),
 plane_height_p5_p50_p95=np.percentile(heights,[5,50,95]).tolist(),
 plane_tilt_p5_p50_p95=np.percentile(tilts,[5,50,95]).tolist(),
 inferred_extrinsic_translation_p5_p95=np.percentile(offsets,[5,95],axis=0).tolist(),
 consensus_cleaned_points=len(cloud.points),audit=audit,
 limitations=['Horizontal surfaces may be desks; this is not a labeled floor calibration.',
 'Saved pose interpolation may differ from acquisition pose; extrinsic variation alone does not prove TF error.',
 'Consensus output does not correct global corridor angles.'])
(out/'floor_pose_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='audit'}))
