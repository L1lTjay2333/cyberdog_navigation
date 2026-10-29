"""Offline edge confirmation from neighboring depth views; original poses kept."""
import argparse
from functools import lru_cache
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
import open3d as o3d
from reconstruction_core import checked_pose
from reconstruction_analysis import rotation
from reconstruction_postprocess import read_depth,filter_depth,masked_record
from reconstruction_noise_audit import depth_layers
from reconstruction_tsdf import fuse_depth_records
from reconstruction_offline_worker import preview_mesh


def world_points(record,depth,mask):
    v,u=np.nonzero(mask);z=depth[v,u];fx,fy,cx,cy=record['intrinsics']
    optical=np.column_stack(((u-cx)*z/fx,(v-cy)*z/fy,z))
    t,_=checked_pose(record)
    return optical@rotation(record).T+t


def view_evidence(points,record,depth):
    """Occlusion is unknown; a point in observed free space is a conflict."""
    t,_=checked_pose(record);p=(points-t)@rotation(record)
    z=p[:,2];valid=np.isfinite(p).all(axis=1)&(z>=.15)&(z<=4.)
    fx,fy,cx,cy=record['intrinsics'];u=np.zeros(len(p),dtype=int);v=u.copy()
    u[valid]=np.rint(p[valid,0]*fx/z[valid]+cx).astype(int)
    v[valid]=np.rint(p[valid,1]*fy/z[valid]+cy).astype(int)
    valid&=(u>=0)&(u<record['width'])&(v>=0)&(v<record['height'])
    measured=np.zeros(len(p));measured[valid]=depth[v[valid],u[valid]]
    valid&=(measured>=.15)&(measured<=4.)
    tolerance=.015+.005*z
    agree=valid&(abs(measured-z)<=tolerance)
    conflict=valid&(measured-z>tolerance)
    return agree,conflict


def confirm_edges(points,neighbors):
    votes=np.zeros(len(points),dtype=int);conflicts=votes.copy()
    seen=set()
    for record,depth in neighbors:
        if record['stamp'] in seen:raise ValueError('Duplicate supporting timestamp')
        seen.add(record['stamp'])
        agree,conflict=view_evidence(points,record,depth);votes+=agree;conflicts+=conflict
    return (votes>=2)&(conflicts==0),votes,conflicts


def build(session,output):
    session,output=Path(session),Path(output)
    if output.exists():raise FileExistsError('Use new output directory')
    archive=session/'frames.jsonl'
    if archive.stat().st_size>256*1024*1024:raise ValueError('Archive size limit')
    data=archive.read_bytes();source_hash=hashlib.sha256(data).hexdigest()
    frames=[json.loads(line) for line in data.splitlines()]
    if not 3<=len(frames)<=900:raise ValueError('Expected 3 to 900 frames')
    if len({(f['frame'],f['source_frame']) for f in frames})!=1:raise ValueError('Frame changed')
    if any(b['stamp']<=a['stamp'] for a,b in zip(frames,frames[1:])):raise ValueError('Timestamps not increasing')
    @lru_cache(maxsize=7)
    def load(i):
        image,depth=read_depth(frames[i]);filtered,_=filter_depth(depth)
        return image,depth,filtered
    output.mkdir(parents=True,exist_ok=False);derived=[];rows=[];uncertain=[]
    with (output/'frames_confirmed.jsonl').open('x',encoding='utf-8') as stream:
        for i,record in enumerate(frames):
            image,depth,filtered=load(i);interior,edge,_=depth_layers(depth)
            points=world_points(record,depth,edge)
            neighbors=[(frames[j],load(j)[2]) for j in range(max(0,i-2),min(len(frames),i+3))
                       if j!=i and abs(frames[j]['stamp']-record['stamp'])<=1.]
            confirmed,votes,conflicts=confirm_edges(points,neighbors)
            keep=interior.copy();keep[edge]=confirmed
            candidate=masked_record(record,image,keep);derived.append(candidate)
            stream.write(json.dumps(candidate,separators=(',',':'))+'\n')
            # Retain a bounded, deterministic visualization of uncertain edges.
            pending=points[~confirmed];uncertain.append(pending[::max(1,len(pending)//200)])
            rows.append({'frame':i+1,'interior':int(interior.sum()),'edge':len(points),
                         'confirmed_edge':int(confirmed.sum()),'uncertain_edge':int((~confirmed).sum()),
                         'conflicting_edge':int((conflicts>0).sum())})
    mesh,_,fusion=fuse_depth_records(derived)
    if not o3d.io.write_triangle_mesh(str(output/'confirmed.ply'),mesh):raise OSError('Mesh write failed')
    descriptor=preview_mesh(output/'confirmed.ply',output,'confirmed')
    pending=np.concatenate(uncertain);np.savez_compressed(output/'uncertain_edges_sample.npz',points=pending)
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pending));o3d.io.write_point_cloud(str(output/'uncertain_edges_sample.ply'),cloud)
    result={'frames':len(frames),'source_sha256':source_hash,'totals':{k:sum(r[k] for r in rows) for k in rows[0] if k!='frame'},
            'rows':rows,'fusion':fusion,'preview':descriptor,'pose_correction':False,'online_enabled':False,
            'limitations':['Two neighboring timestamps are not independent metrology or proof of a true thin structure.',
                           'Pose errors and occlusions can make real edges uncertain; this is NOT the default model.',
                           'Original frames retain all edge observations; uncertain cloud is only a deterministic sample.',
                           'Interior pixels are unchanged. No missing seat surfaces are invented.']}
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    (output/'edge_report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


def install_comparison(output,baseline_run,web_output):
    output,baseline_run,web_output=map(Path,(output,baseline_run,web_output))
    report=json.loads((output/'edge_report.json').read_text(encoding='utf-8'))
    baseline=json.loads((baseline_run/'result.json').read_text(encoding='utf-8'))
    if report['source_sha256']!=baseline['source_sha256']:raise ValueError('Mismatched baseline')
    web_output.mkdir(parents=True,exist_ok=False)
    for source,name in [(output/'confirmed.bin','confirmed.bin'),(output/'confirmed.ply','confirmed.ply'),
                        (output/'edge_report.json','edge_report.json'),(baseline_run/'observed.bin','baseline.bin')]:
        shutil.copyfile(source,web_output/name)
    with np.load(output/'uncertain_edges_sample.npz',allow_pickle=False) as saved:
        points=saved['points'];selected=points[np.linspace(0,len(points)-1,min(len(points),30000)).astype(int)]
    selected.astype('<f4').tofile(web_output/'pending.bin')
    view={'baseline':dict(baseline['previews']['observed'],file='baseline.bin'),
          'confirmed':report['preview'],'pending':{'file':'pending.bin','position_count':len(selected)},'totals':report['totals']}
    (web_output/'view.json').write_text(json.dumps(view,ensure_ascii=False),encoding='utf-8')
    (web_output/'index.html').write_bytes(Path(__file__).with_name('edge_support_viewer.html').read_bytes())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session');parser.add_argument('output')
    args=parser.parse_args();r=build(args.session,args.output)
    print(json.dumps({'totals':r['totals'],'mesh':r['fusion']['all_observations']},indent=2))
