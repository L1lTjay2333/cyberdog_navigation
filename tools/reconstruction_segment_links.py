"""Offline adjacent segment link screening. Never applies a room pose graph."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruction_turn_audit import load_views
from reconstruction_multiscale import propose
from reconstruction_registration import apply_transform,sampled_cloud


def check_link(source,target,solver=None):
    solver=solver or propose
    forward=solver(source,target);reverse=solver(target,source)
    result={'accepted':False,'forward':forward,'reverse':reverse}
    if not forward['qualified'] or not reverse['qualified']:
        return dict(result,reason='directional_check_failed')
    cycle=np.array(reverse['transform'])@np.array(forward['transform'])
    center=np.mean(source,axis=0)
    shift=float(np.linalg.norm(apply_transform(center[None],cycle)[0]-center))
    angle=float(np.degrees(Rotation.from_matrix(cycle[:3,:3]).magnitude()))
    accepted=shift<=.01 and angle<=1.
    return dict(result,accepted=accepted,reason='qualified_local_link' if accepted else 'cycle_inconsistent',
                cycle_shift_m=shift,cycle_angle_deg=angle)


def audit_links(session,segments_file,output):
    session,segments_file,output=Path(session),Path(segments_file),Path(output)
    if output.exists():raise FileExistsError('Use a new output directory')
    frames,clouds,_,source_hash=load_views(session)
    info=json.loads(segments_file.read_text(encoding='utf-8'))
    if info['source_sha256']!=source_hash:raise ValueError('Segments use a different capture')
    groups=info['segments'];rows=[]
    for left,right in zip(groups,groups[1:]):
        def boundary(group,tail):
            indices=list(range(group['first_frame']-1,group['last_frame']))
            pairs=list(zip(indices,group['corrections']))
            pairs=pairs[-3:] if tail else pairs[:3]
            return sampled_cloud(np.concatenate([apply_transform(clouds[i],np.array(delta)) for i,delta in pairs]))
        source,target=boundary(right,False),boundary(left,True)
        row=check_link(source,target)
        rows.append(dict(row,left=left['segment'],right=right['segment']))
    if hashlib.sha256((session/'frames.jsonl').read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    result={'frames':len(frames),'segments':len(groups),'source_sha256':source_hash,'links':rows,
            'qualified_links':sum(row['accepted'] for row in rows),
            'adjacent_chain_connected':all(row['accepted'] for row in rows),
            'pose_graph_applied':False,'absolute_accuracy_validated':False,
            'limitations':['Only adjacent boundaries tested; a failed link does not prove no alternative overlap exists.',
                           'Cycle consistency is necessary screening, not independent metric ground truth.',
                           'No room model or source pose modified.']}
    output.mkdir(parents=True,exist_ok=False)
    (output/'links.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session');parser.add_argument('segments');parser.add_argument('output')
    args=parser.parse_args();report=audit_links(args.session,args.segments,args.output)
    print(json.dumps({key:value for key,value in report.items() if key!='links'},ensure_ascii=False,indent=2))
