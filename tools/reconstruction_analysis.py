"""Offline diagnostics for saved local depth/pose captures. No robot access."""
import argparse
import base64
import json
from pathlib import Path
import zlib
import numpy as np
from reconstruction_core import checked_pose, decode_depth_frame


def distribution(values):
    values=np.asarray(values,dtype=float)
    values=values[np.isfinite(values)]
    if not len(values):return {'count':0}
    return {'count':len(values),'min':float(values.min()),'median':float(np.median(values)),
            'p90':float(np.percentile(values,90)),'p95':float(np.percentile(values,95)),
            'max':float(values.max())}


def rotation(frame):
    _,q=checked_pose(frame);x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def compare_frames(reference, source):
    """Reproject source into a reference depth view; no ICP or pose correction.

    Nearest-pixel depth residuals include occlusion, sampling and viewpoint
    differences; they are consistency diagnostics, not absolute accuracy.
    """
    if reference['frame']!=source['frame'] or reference['source_frame']!=source['source_frame']:
        raise ValueError('Cannot compare different coordinate frames or cameras')
    reference_points=decode_depth_frame(reference)
    source_points=decode_depth_frame(source)
    t,_=checked_pose(reference)
    optical=(source_points-t).dot(rotation(reference))
    positive=optical[:,2]>.15;optical=optical[positive]
    fx,fy,cx,cy=reference['intrinsics']
    uv=np.rint(optical[:,:2]/optical[:,2,None]*[fx,fy]+[cx,cy]).astype(np.int64)
    w,h=reference['width'],reference['height']
    inside=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    uv=uv[inside];optical=optical[inside]
    size=2 if reference['encoding']=='16UC1' else 4
    dtype=('>' if reference['bigendian'] else '<')+('u2' if size==2 else 'f4')
    # decode_depth_frame validated compressed size, calibration and layout above.
    raw=zlib.decompress(base64.b64decode(reference['depth_zlib_b64']))
    image=np.ndarray((h,w),dtype=dtype,buffer=raw,strides=(reference['step'],size))
    measured=image[uv[:,1],uv[:,0]].astype(float)*(.001 if size==2 else 1.)
    valid=np.isfinite(measured)&(measured>=.15)&(measured<=4.)
    measured=measured[valid];projected=optical[valid,2]
    residual=measured-projected
    result={'reference_stamp':reference['stamp'],'source_stamp':source['stamp'],
            'source_points':len(source_points),'comparable_pixels':len(residual),
            'comparable_fraction':len(residual)/float(len(source_points)) if len(source_points) else 0,
            'signed_depth_residual_m':distribution(residual),'absolute_depth_residual_m':distribution(abs(residual)),
            'within_1cm_fraction':float(np.mean(abs(residual)<=.01)) if len(residual) else None,
            'within_2cm_fraction':float(np.mean(abs(residual)<=.02)) if len(residual) else None,
            'residual_by_projected_depth':{}}
    for lo,hi in [(.15,.5),(.5,1.),(1.,2.),(2.,4.)]:
        mask=(projected>=lo)&(projected<hi)
        result['residual_by_projected_depth']['%g-%gm'%(lo,hi)]=distribution(abs(residual[mask]))
    try:
        from scipy.spatial import cKDTree
        distances,_=cKDTree(reference_points).query(source_points,k=1)
        result['nearest_surface_distance_m']=distribution(distances)
    except ImportError:
        result['nearest_surface_distance_m']={'unavailable':'SciPy is not installed'}
    return result


def replay_voxel_counts(frames, sizes=(.01,.015,.02,.03)):
    points=[decode_depth_frame(frame) for frame in frames]
    results=[]
    for size in sizes:
        cells={}
        for cloud in points:
            keys=np.unique(np.floor(cloud/size).astype(np.int64),axis=0)
            for k in keys:
                key=tuple(k);cells[key]=cells.get(key,0)+1
        results.append({'voxel_m':size,'frames_replayed':len(frames),'voxels':len(cells),
                        'supported_by_2_frames':sum(v>=2 for v in cells.values()),
                        'single_frame_voxels':sum(v==1 for v in cells.values())})
    return results


def analyze_session(folder):
    folder=Path(folder)
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    samples=json.loads((folder/'samples.json').read_text(encoding='utf-8'))
    poses=summary.get('trajectory') or []
    positions=np.asarray([p['position'] for p in poses],dtype=float)
    stamps=np.asarray([p['stamp'] for p in poses],dtype=float)
    frame_files=sorted(folder.glob('frame_*.json'))
    frames=[json.loads(p.read_text(encoding='utf-8')) for p in frame_files]
    motion={}
    if len(poses)>1:
        delta=np.linalg.norm(np.diff(positions,axis=0),axis=1);dt=np.diff(stamps)
        if (dt<=0).any():raise ValueError('Non-monotonic saved trajectory')
        qs=np.asarray([p['quaternion'] for p in poses],dtype=float)
        qs=qs/np.linalg.norm(qs,axis=1)[:,None]
        angles=2*np.arccos(np.clip(abs(np.sum(qs[1:]*qs[:-1],axis=1)),0,1))
        motion={'accepted_span_s':float(stamps[-1]-stamps[0]),
                'net_camera_displacement_m':float(np.linalg.norm(positions[-1]-positions[0])),
                'accumulated_camera_displacement_m':float(delta.sum()),
                'step_translation_m':distribution(delta),'source_interval_s':distribution(dt),
                'estimated_step_speed_mps':distribution(delta/dt),
                'step_angle_deg':distribution(np.degrees(angles)),
                'net_angle_deg':float(np.degrees(2*np.arccos(np.clip(abs(qs[0].dot(qs[-1])),0,1))))}
    rows=[]
    pairs=list(zip(range(len(frames)-1),range(1,len(frames))))
    if len(frames)>2:pairs.extend([(0,len(frames)-1),(len(frames)-1,0)])
    for i,j in pairs:
        rows.append(dict(compare_frames(frames[i],frames[j]),reference_file=frame_files[i].name,source_file=frame_files[j].name))
    return {'session':folder.name,'accepted_frames':summary['accepted_frames'],
            'stable_points':summary['stable_points'],'total_voxels':summary['voxels'],
            'single_frame_voxels':summary['voxels']-summary['stable_points'],
            'capacity_fraction':summary['voxels']/float(summary['max_voxels']),
            'termination':summary.get('error'),'motion':motion,
            'source_age_s':distribution([s['source_age_s'] for s in samples if s.get('source_age_s') is not None]),
            'saved_keyframes':len(frames),'pair_consistency':rows,'voxel_replay':replay_voxel_counts(frames),
            'user_observation':'User reported no obvious double-edge ghosting; visual field feedback, not metrology.',
            'limitations':['No independent ruler ground truth; no absolute accuracy validation.',
                          'Saved keyframes are a subset, not full replay of all accepted frames.',
                          'Depth reprojection includes occlusions, pixel rounding, motion and surface differences.',
                          'Nearest-neighbor distances can understate tangential slip on a flat wall.',
                          'Odometry-derived travel includes sway/noise and is not measured robot travel.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    args=parser.parse_args()
    report=analyze_session(args.session)
    (args.session/'offline_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('pair_consistency','limitations')},ensure_ascii=True,indent=2))
    for row in report['pair_consistency']:
        print(json.dumps({k:row[k] for k in ('reference_file','source_file','comparable_fraction','absolute_depth_residual_m','within_2cm_fraction','nearest_surface_distance_m')},ensure_ascii=True))
