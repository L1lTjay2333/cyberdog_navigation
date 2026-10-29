"""Compare filtered adjacent views and a guarded trajectory, offline only."""
import argparse
import hashlib
import json
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruction_core import checked_pose,decode_depth_frame
from reconstruction_postprocess import read_depth,filter_depth,masked_record
from reconstruction_registration import sampled_cloud,apply_transform
from reconstruction_multiscale import propose,track_step
from benchmark_open3d_registration import assess_candidate


def load_views(session,max_depth=4.):
    if not np.isfinite(max_depth) or not .3<=max_depth<=4.:raise ValueError('Depth limit must be 0.3 to 4 m')
    session=Path(session)
    archive=session/'frames.jsonl'
    if not 0<archive.stat().st_size<=256*1024*1024:raise ValueError('Archive size limit')
    before=hashlib.sha256(archive.read_bytes()).hexdigest()
    frames=[]
    with archive.open(encoding='utf-8') as stream:
        for line in stream:
            frames.append(json.loads(line))
            if len(frames)>900:raise ValueError('Frame limit')
    if len(frames)<2:raise ValueError('Need at least two views')
    if len({(f['frame'],f['source_frame']) for f in frames})!=1:raise ValueError('Coordinate frame changed')
    if any(b['stamp']<=a['stamp'] for a,b in zip(frames,frames[1:])):raise ValueError('Unordered timestamps')
    clouds=[];poses=[]
    for record in frames:
        t,q=checked_pose(record);poses.append((t,Rotation.from_quat(q)))
        image,depth=read_depth(record);filtered,_=filter_depth(depth)
        clouds.append(sampled_cloud(decode_depth_frame(masked_record(record,image,(filtered>0)&(filtered<=max_depth)))))
    return frames,clouds,poses,before


def audit(session,output,max_depth=4.):
    session,output=Path(session),Path(output)
    frames,clouds,poses,before=load_views(session,max_depth=max_depth)
    archive=session/'frames.jsonl'
    rows=[];prior=np.eye(4);references=[clouds[0]];tracking=True;tracked=1;corrections=[prior.tolist()]
    for index in range(1,len(frames)):
        src,dst=clouds[index],clouds[index-1]
        baseline=assess_candidate(src,dst,np.eye(4));candidate=propose(src,dst)
        row={'frame':index+1,'stamp':frames[index]['stamp'],
             'turn_deg':float(np.degrees((poses[index-1][1].inv()*poses[index][1]).magnitude())),
             'translation_m':float(np.linalg.norm(poses[index][0]-poses[index-1][0])),
             'baseline':baseline,'multiscale':candidate}
        if tracking:
            quality,combined=track_step(src,prior,references)
            if combined is not None:
                prior=combined;references.append(sampled_cloud(apply_transform(src,prior)));references=references[-3:]
                corrections.append(prior.tolist());tracked+=1
            tracking=quality['qualified'];row['trajectory']=quality
        rows.append(row)
    after=hashlib.sha256(archive.read_bytes()).hexdigest()
    if after!=before:raise ValueError('Source archive changed')
    result={'frames':len(frames),'source_sha256':before,'source_unchanged':True,'max_depth_m':max_depth,
            'baseline_qualified_pairs':sum(r['baseline']['qualified'] for r in rows),
            'multiscale_qualified_pairs':sum(r['multiscale']['qualified'] for r in rows),
            'candidate_reasons':dict(Counter(r['multiscale']['reason'] for r in rows)),
            'consecutive_tracked_frames':tracked,'full_trajectory_qualified':tracked==len(frames),
            'corrections_for_qualified_prefix':corrections,'rows':rows,
            'absolute_accuracy_validated':False,'online_enabled':False,
            'note':'Pairwise improvement does not certify a cumulative trajectory or absolute accuracy; no raw poses changed.'}
    output.mkdir(parents=True,exist_ok=False)
    (output/'turn_report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session');parser.add_argument('output')
    parser.add_argument('--max-depth',type=float,default=4.)
    args=parser.parse_args();report=audit(args.session,args.output,max_depth=args.max_depth)
    print(json.dumps({key:value for key,value in report.items() if key not in ('rows','corrections_for_qualified_prefix')},ensure_ascii=False,indent=2))
