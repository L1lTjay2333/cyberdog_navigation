"""Offline surface reconstruction from PC archived aligned XYZ. No robot access."""
import json,time,hashlib,argparse
from pathlib import Path
import numpy as np
import open3d as o3d

def run(root,out,cached=False):
 out.mkdir(parents=True,exist_ok=cached)
 latest=json.loads((root/'latest.json').read_text(encoding='utf-8'));key=latest['key']
 def progress(phase,**kw):
  (out/'progress.json').write_text(json.dumps(dict(phase=phase,**kw)),encoding='utf-8')
 progress('snapshot');seen=set();cells={};sources=[];sizes=[];centroids=[];times=[]
 if cached:
  prior=json.loads((out/'result.json').read_text(encoding='utf-8'));key=prior['key'];latest['frame']=prior['frame']
  with np.load(out/'voxel_consensus.npz') as data:
   cells={tuple(np.floor(p/.05).astype(np.int64)):[p*int(n),int(n)] for p,n in zip(data['points'],data['frame_support'])}
  sources=prior['sources'];seen=set(range(prior['frames']));times=[0,prior['duration_s']]
 # Freeze byte lengths, copy complete records, never modify live input.
 for path in ([] if cached else sorted(root.glob('session_*/frames.jsonl'))):
  size=path.stat().st_size;copied=0;sha=hashlib.sha256();target=out/(path.parent.name+'.jsonl')
  with path.open('rb') as f,target.open('wb') as saved:
   while f.tell()<size:
    line=f.readline(min(size-f.tell(),4*1024*1024))
    if not line.endswith(b'\n'):break
    r=json.loads(line)
    if r['key']!=key or r['stamp'] in seen:continue
    seen.add(r['stamp']);saved.write(line);sha.update(line);copied+=1
    p=np.asarray(r['points_xyz'],dtype=float);p=p[np.isfinite(p).all(axis=1)]
    if not len(p):continue
    sizes.append(len(p));centroids.append(np.median(p,axis=0).tolist());times.append(r['stamp'])
    # Each frame contributes at most once to a 5cm cell.
    v=np.floor(p/.05).astype(np.int64);_,idx=np.unique(v,axis=0,return_index=True)
    for j in idx:
     k=tuple(v[j]);a=cells.get(k)
     if a is None:cells[k]=[p[j].copy(),1]
     else:a[0]+=p[j];a[1]+=1
  if copied:sources.append(dict(file=target.name,frames=copied,bytes=target.stat().st_size,sha256=sha.hexdigest()))
  else:target.unlink()
 progress('consensus',frames=len(seen),voxels=len(cells))
 allp=np.array([a[0]/a[1] for a in cells.values()]);counts=np.array([a[1] for a in cells.values()])
 np.savez_compressed(out/'voxel_consensus.npz',points=allp,frame_support=counts)
 # Keep single-view geometry in an explicit raw comparison, not secretly erase it.
 raw=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(allp));o3d.io.write_point_cloud(str(out/'raw.ply'),raw)
 support=counts>=3;p=allp[support]
 if len(p)<100:raise ValueError('Too few repeated observations')
 cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
 cloud,idx=cloud.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)
 p=np.asarray(cloud.points).copy();before=p.copy();remaining=np.arange(len(p));planes=[]
 o3d.utility.random.seed(11)
 # Fit only strongly supported planar patches; snap at most 2cm. No cuboid priors.
 for _ in range(10):
  if len(remaining)<1200:break
  sub=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p[remaining]))
  equation,inliers=sub.segment_plane(distance_threshold=.02,ransac_n=3,num_iterations=500)
  if len(inliers)<max(1000,int(len(p)*.01)):break
  ids=remaining[np.asarray(inliers)];n=np.asarray(equation[:3]);d=equation[3];res=p[ids]@n+d
  # Only refine existing samples. Do not fill planar hulls or invent unseen surfaces.
  keep=np.abs(res)<=.02;ids=ids[keep];res=res[keep]
  p[ids]-=res[:,None]*n
  planes.append(dict(points=len(ids),normal=n.tolist(),offset=d,rms_before=float(np.sqrt(np.mean(res**2)))))
  mask=np.ones(len(remaining),bool);mask[inliers]=False;remaining=remaining[mask]
 cloud.points=o3d.utility.Vector3dVector(p)
 o3d.io.write_point_cloud(str(out/'processed.ply'),cloud)
 progress('surface',points=len(p),planes=len(planes))
 cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.18,max_nn=30))
 cloud.orient_normals_consistent_tangent_plane(20)
 mesh=o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud,o3d.utility.DoubleVector([.06,.09,.12]))
 vertices=np.asarray(mesh.vertices);faces=np.asarray(mesh.triangles)
 if len(faces):
  lengths=np.linalg.norm(vertices[faces]-np.roll(vertices[faces],1,axis=1),axis=2)
  mesh.remove_triangles_by_mask((lengths>.20).any(axis=1))
 mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
 mesh.paint_uniform_color([.65,.65,.65]);o3d.io.write_triangle_mesh(str(out/'gray.ply'),mesh)
 o3d.io.write_triangle_mesh(str(out/'gray.obj'),mesh,write_vertex_colors=False)
 preview=mesh.simplify_quadric_decimation(180000) if len(mesh.triangles)>180000 else mesh
 np.asarray(preview.vertices)[np.asarray(preview.triangles)].astype('<f4').tofile(out/'gray.bin')
 for name,points in [('raw',allp),('processed',p)]:points.astype('<f4').tofile(out/(name+'.bin'))
 report=dict(key=key,frame=latest['frame'],frames=len(seen),duration_s=max(times)-min(times),sources=sources,
  raw_points=len(allp),consensus_points=int(support.sum()),processed_points=len(p),triangles=len(mesh.triangles),
  preview_triangles=len(preview.triangles),bounds=[allp.min(axis=0).tolist(),allp.max(axis=0).tolist()],planes=planes,
  max_adjustment_m=float(np.linalg.norm(p-before,axis=1).max()),
  support_threshold=3,voxel_m=.05,mesh_max_edge_m=.20,
  limitations=['Aligned XYZ only: raw depth, camera poses and RGB were not archived in this mode.',
   'No pose-graph optimization or loop-closure correction; drift may remain.',
   'Surface gaps are retained. No watertight completion. Centroid is not robot trajectory.'])
 (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 progress('done',points=len(p),triangles=len(mesh.triangles));print(json.dumps(report),flush=True)

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('source',type=Path);a.add_argument('output',type=Path);a.add_argument('--reprocess',action='store_true');args=a.parse_args();run(args.source,args.output,args.reprocess)
