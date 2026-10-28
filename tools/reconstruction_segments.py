"""Experimental independently anchored surface segments, not a stitched room."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from reconstruction_core import checked_pose
from reconstruction_turn_audit import load_views
from reconstruction_multiscale import split_tracks
from reconstruction_tsdf import fuse_depth_records
from reconstruction_offline_worker import preview_mesh


def corrected_record(record,correction):
    t,q=checked_pose(record)
    correction=np.asarray(correction,dtype=float)
    if (correction.shape!=(4,4) or not np.isfinite(correction).all() or
        not np.allclose(correction[3],[0,0,0,1]) or
        not np.allclose(correction[:3,:3].T@correction[:3,:3],np.eye(3),atol=1e-7) or
        not np.isclose(np.linalg.det(correction[:3,:3]),1.)):
        raise ValueError('Expected rigid correction')
    result=dict(record)
    result['translation']=(correction[:3,:3]@t+correction[:3,3]).tolist()
    result['quaternion']=Rotation.from_matrix(correction[:3,:3]@Rotation.from_quat(q).as_matrix()).as_quat().tolist()
    return result


def build_segments(session,output):
    session,output=Path(session),Path(output)
    if output.exists():raise FileExistsError('Use a new output directory')
    frames,clouds,_,source_hash=load_views(session)
    groups=split_tracks(clouds);output.mkdir(parents=True,exist_ok=False)
    rows=[]
    for number,group in enumerate(groups,1):
        folder=output/('segment_%03d'%number);folder.mkdir()
        records=[frames[i] for i in group['indices']]
        corrected=[corrected_record(frame,delta) for frame,delta in zip(records,group['corrections'])]
        with (folder/'frames_corrected.jsonl').open('x',encoding='utf-8') as stream:
            for record in corrected:stream.write(json.dumps(record,separators=(',',':'))+'\n')
        row={'segment':number,'first_frame':group['indices'][0]+1,'last_frame':group['indices'][-1]+1,
             'frames':len(records),'start_reason':group['start_reason'],'previews':{},
             'checks':group['checks'],'incoming_rejection':group.get('rejected_incoming_edge'),
             'corrections':[delta.tolist() for delta in group['corrections']]}
        for label,input_records in (('baseline',records),('refined',corrected)):
            mesh,_,report=fuse_depth_records(input_records)
            # fuse_depth_records has no trajectory knowledge: override its
            # unchanged-pose metadata for this explicitly derived experiment.
            changed=label=='refined' and any(not np.allclose(delta,np.eye(4),atol=1e-10) for delta in group['corrections'])
            report.update(pose_correction=changed,pose_refinement_enabled=label=='refined',independent_segment=True,
                          limitations=['Independent local segment; inter-segment alignment is unvalidated.',
                                       'No absolute metric ground truth or loop closure.'])
            path=folder/(label+'.ply')
            if len(mesh.triangles):
                if not o3d.io.write_triangle_mesh(str(path),mesh):raise OSError('Mesh export failed')
                descriptor=preview_mesh(path,folder,label)
                descriptor['file']=folder.name+'/'+descriptor['file']
                row['previews'][label]=descriptor
            row[label+'_report']=report
        (folder/'segment_report.json').write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf-8')
        rows.append(row)
    if hashlib.sha256((session/'frames.jsonl').read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    indices=[i for group in groups for i in group['indices']]
    if indices!=list(range(len(frames))):raise ValueError('Lost or duplicated source frame')
    result={'source_sha256':source_hash,'source_frames':len(frames),'segments':rows,
            'all_frames_preserved_once':True,'merged_room_model':False,'absolute_accuracy_validated':False,
            'note':'Each segment restarts at the original pose; do not merge segments as a corrected room.'}
    (output/'segments.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'index.html').write_bytes(Path(__file__).with_name('segment_viewer.html').read_bytes())
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session');parser.add_argument('output')
    args=parser.parse_args();r=build_segments(args.session,args.output)
    print(json.dumps({'frames':r['source_frames'],'segments':[(s['first_frame'],s['last_frame'],s['start_reason']) for s in r['segments']],
                      'all_frames_preserved_once':r['all_frames_preserved_once']},ensure_ascii=False))
