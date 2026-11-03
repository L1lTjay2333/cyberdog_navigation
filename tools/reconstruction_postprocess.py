"""Offline raw-depth/point preprocessing BEFORE meshing. No mesh polishing."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import time
import zlib
import numpy as np
from scipy.spatial import cKDTree
from reconstruction_core import checked_pose,decode_depth_frame


def read_depth(record):
    decode_depth_frame(record,stride=16)  # Reuse bounded payload and pose validation.
    size=2 if record['encoding']=='16UC1' else 4
    dtype=('>' if record['bigendian'] else '<')+('u2' if size==2 else 'f4')
    raw=zlib.decompress(base64.b64decode(record['depth_zlib_b64'],validate=True))
    image=np.ndarray((record['height'],record['width']),dtype=dtype,buffer=raw,
                     strides=(record['step'],size)).copy()
    depth=image.astype(float)*(.001 if size==2 else 1.)
    return image,depth


def filter_depth(depth):
    depth=np.asarray(depth,dtype=float)
    if depth.ndim!=2 or not depth.size:raise ValueError('Expected nonempty depth image')
    valid=np.isfinite(depth)&(depth>=.15)&(depth<=4.)
    padded=np.pad(depth,1,constant_values=np.nan)
    neighbors=np.zeros(depth.shape,dtype=np.int16)
    with np.errstate(invalid='ignore'):
        tolerance=.03+.005*depth
        for dy in range(3):
            for dx in range(3):
                if dy==dx==1:continue
                other=padded[dy:dy+depth.shape[0],dx:dx+depth.shape[1]]
                neighbors+=(np.isfinite(other)&(other>=.15)&(other<=4.)&(abs(other-depth)<=tolerance))
    keep=valid&(neighbors>=2)
    result=np.where(keep,depth,0.)
    return result,{'raw_valid_pixels':int(valid.sum()),'retained_pixels':int(keep.sum()),
                   'isolated_removed':int((valid&~keep).sum())}


def masked_record(record,image,keep):
    result=dict(record)
    dtype='<u2' if record['encoding']=='16UC1' else '<f4'
    output=image.astype(dtype);output[~keep]=0
    result.update(step=record['width']*output.dtype.itemsize,bigendian=False,
                  depth_zlib_b64=base64.b64encode(zlib.compress(output.tobytes(),1)).decode('ascii'))
    return result


class VoxelConsensus:
    def __init__(self,voxel=.02,max_cells=1000000):
        if not np.isfinite(voxel) or not .005<=voxel<=.05:raise ValueError('Invalid voxel size')
        self.voxel,self.max_cells=voxel,max_cells
        self.cells={}

    def keys(self,points):
        scaled=np.floor(np.asarray(points)/self.voxel)
        if not np.isfinite(scaled).all() or np.any(abs(scaled)>np.iinfo(np.int64).max//2):
            raise ValueError('Invalid voxel coordinates')
        return scaled.astype(np.int64)

    def add(self,points):
        if not len(points):return
        keys,inverse,counts=np.unique(self.keys(points),axis=0,return_inverse=True,return_counts=True)
        if len(self.cells)+sum(tuple(k) not in self.cells for k in keys)>self.max_cells:
            raise ValueError('Offline consensus capacity exceeded; originals unchanged')
        centers=np.column_stack([np.bincount(inverse,weights=points[:,i]) for i in range(3)])/counts[:,None]
        for key,center in zip(keys,centers):
            cell=self.cells.setdefault(tuple(key),[np.zeros(3),0])
            cell[0]+=center;cell[1]+=1  # One vote and equal weight per frame.

    def arrays(self):
        keys=np.asarray(list(self.cells),dtype=np.int64).reshape((-1,3))
        points=np.asarray([c[0]/c[1] for c in self.cells.values()],dtype=float).reshape((-1,3))
        support=np.asarray([c[1] for c in self.cells.values()],dtype=np.uint32)
        return points,keys,support


def select_consensus(points,support,minimum_neighbors=3):
    eligible=np.asarray(support)>=2
    keep=np.zeros(len(points),dtype=bool)
    if not eligible.any():return keep
    selected=points[eligible];tree=cKDTree(selected)
    neighbors=tree.query_ball_point(selected,.05,return_length=True)-1
    keep[np.flatnonzero(eligible)]=neighbors>=minimum_neighbors
    return keep


def write_cloud(path,points,support):
    with path.open('x',encoding='ascii') as stream:
        stream.write('ply\nformat ascii 1.0\ncomment processed observations; meters; accuracy unvalidated\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uint frame_support\nend_header\n'%len(points))
        for point,votes in zip(points,support):stream.write('%.7f %.7f %.7f %d\n'%(*point,int(votes)))


def process_capture(session,output):
    session,output=Path(session),Path(output)
    archive=session/'frames.jsonl'
    if not archive.is_file() or archive.stat().st_size>256*1024*1024:
        raise ValueError('Expected bounded original frame archive')
    output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    source_hash=hashlib.sha256(archive.read_bytes()).hexdigest()
    model=VoxelConsensus();rows=[];stamp=0.;origin=None
    totals={'raw_valid_pixels':0,'retained_pixels':0,'isolated_removed':0,
            'raw_sampled_observations':0,'filtered_sampled_observations':0}
    spatial_archive=output/'frames_spatial.jsonl'
    with archive.open(encoding='utf-8') as source,spatial_archive.open('x',encoding='utf-8') as spatial:
        for index,line in enumerate(source,1):
            if index>900:raise ValueError('Offline trial supports at most 900 frames')
            record=json.loads(line);checked_pose(record)
            current=(record['frame'],record['source_frame'])
            if origin is not None and current!=origin:raise ValueError('Coordinate frame changed')
            if record['stamp']<=stamp:raise ValueError('Non-increasing timestamps')
            origin,stamp=current,record['stamp']
            image,depth=read_depth(record);filtered,stats=filter_depth(depth)
            updated=masked_record(record,image,filtered>0)
            points=decode_depth_frame(updated)
            valid=np.isfinite(depth)&(depth>=.15)&(depth<=4.)
            stats.update(raw_sampled_observations=int(valid[::4,::4].sum()),filtered_sampled_observations=len(points))
            for key in totals:totals[key]+=stats[key]
            model.add(points);rows.append(stats)
            spatial.write(json.dumps(updated,separators=(',',':'),allow_nan=False)+'\n')
    if not rows:raise ValueError('Empty frame archive')
    points,keys,support=model.arrays()
    keep=select_consensus(points,support)
    if not keep.any():raise ValueError('No confirmed points remain; original input preserved')
    retained_keys=set(map(tuple,keys[keep]))
    output_archive=output/'frames.jsonl';used_observations=0
    # Export derived, stride-4 depth samples accepted by global consensus. This
    # masks unsupported samples; surviving measured depth/pose values are unchanged.
    with spatial_archive.open(encoding='utf-8') as spatial,output_archive.open('x',encoding='utf-8') as target:
        for line in spatial:
            record=json.loads(line);image,depth=read_depth(record)
            z=depth[::4,::4];valid=np.isfinite(z)&(z>=.15)&(z<=4.)
            world=decode_depth_frame(record)
            selected=np.fromiter((tuple(k) in retained_keys for k in model.keys(world)),dtype=bool,count=len(world))
            mask=np.zeros(z.shape,dtype=bool);mask[valid]=selected
            full_mask=np.zeros(depth.shape,dtype=bool);full_mask[::4,::4]=mask
            updated=masked_record(record,image,full_mask)
            updated['postprocess']={'pixel_stride':4,'voxel_m':model.voxel,'minimum_frame_support':2,
                                    'radius_m':.05,'minimum_neighbors':3,'depth_values_smoothed':False}
            used_observations+=int(selected.sum())
            target.write(json.dumps(updated,separators=(',',':'),allow_nan=False)+'\n')
    write_cloud(output/'points_candidates.ply',points,support)
    write_cloud(output/'points_processed.ply',points[keep],support[keep])
    write_cloud(output/'points_excluded.ply',points[~keep],support[~keep])
    np.savez(output/'points_processed.npz',points=points[keep],keys=keys[keep],support=support[keep])
    summary_path=session/'summary.json'
    original=json.loads(summary_path.read_text(encoding='utf-8')) if summary_path.exists() else {}
    info=original.get('raw_archive') or {}
    complete=bool(info.get('complete') and info.get('frames')==original.get('received_frames',original.get('accepted_frames'))==len(rows))
    summary={'accepted_frames':len(rows),'frame':origin[0],'raw_archive':{'complete':True,'frames':len(rows),
             'file':'frames.jsonl','derived':True},'error':None,'source_capture_termination':original.get('error')}
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    report=dict(totals,frames=len(rows),candidate_voxels=len(points),
                single_frame_voxels=int((support<2).sum()),spatially_isolated_confirmed_voxels=int(((support>=2)&~keep).sum()),
                processed_points=int(keep.sum()),excluded_points=int((~keep).sum()),
                retained_frame_observations=used_observations,voxel_m=.02,minimum_frame_support=2,
                neighborhood_radius_m=.05,minimum_neighbors=3,
                source_archive=str(archive.resolve()),source_sha256=source_hash,source_complete_for_accepted_frames=complete,
                source_capture_termination=original.get('error'),processing_seconds=time.perf_counter()-started,
                depth_smoothing=False,mesh_processing=False,pose_correction=False,absolute_accuracy_validated=False,
                limitations=['Single-observation surfaces and small real objects may be excluded; excluded representatives are saved.',
                              'Voxel counts are not noise counts; most reduction is repeat-observation merging.',
                              'Odometry errors larger than the voxel can still produce doubled surfaces.',
                              'Derived depth archive is stride-4 and masked; not original full-resolution sensor depth.'])
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed during processing')
    (output/'processing_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();print(json.dumps(process_capture(args.session,args.output),ensure_ascii=False,indent=2))
