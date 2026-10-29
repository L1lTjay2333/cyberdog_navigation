"""Offline motion/edge ablation. Candidate masks are diagnostic, never defaults."""
import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from reconstruction_core import checked_pose
from reconstruction_postprocess import read_depth,filter_depth,masked_record
from reconstruction_analysis import compare_frames
from reconstruction_tsdf import fuse_depth_records
from reconstruction_offline_worker import preview_mesh


def depth_layers(depth):
    filtered,stats=filter_depth(depth);valid=filtered>0
    padded=np.pad(filtered,1,constant_values=0.)
    boundary=np.zeros(valid.shape,dtype=bool)
    for dy,dx in ((0,1),(1,0),(1,2),(2,1)):
        neighbor=padded[dy:dy+valid.shape[0],dx:dx+valid.shape[1]]
        boundary|=(neighbor<=0)|(abs(neighbor-filtered)>.05+.01*filtered)
    interior=valid&~boundary
    return interior,valid&boundary,stats


def select_windows(frames,count=10):
    if len(frames)<count:raise ValueError('Not enough frames')
    poses=[checked_pose(f) for f in frames];choices={}
    for start in range(len(frames)-count+1):
        part=poses[start:start+count];t=np.array([p[0] for p in part]);rot=Rotation.from_quat([p[1] for p in part])
        distance=float(np.linalg.norm(t-t[0],axis=1).max())
        angle=float(np.degrees((rot[0].inv()*rot).magnitude()).max())
        if frames[start+count-1]['stamp']-frames[start]['stamp']<2:continue
        if 'static' not in choices and distance<=.02 and angle<=2.:
            choices['static']={'start':start,'end':start+count,'max_translation_m':distance,'max_angle_deg':angle}
        if ('moving' not in choices and 'static' in choices and start>=choices['static']['end']
                and (distance>=.15 or angle>=15.)):
            choices['moving']={'start':start,'end':start+count,'max_translation_m':distance,'max_angle_deg':angle}
    if set(choices)!={'static','moving'}:raise ValueError('No qualifying static/moving windows; cannot invent comparison')
    return choices


def build(session,output):
    session,output=Path(session),Path(output)
    if output.exists():raise FileExistsError('Use a new output directory')
    archive=session/'frames.jsonl'
    if archive.stat().st_size>256*1024*1024:raise ValueError('Archive size limit')
    data=archive.read_bytes();source_hash=hashlib.sha256(data).hexdigest()
    frames=[json.loads(line) for line in data.splitlines()]
    if len(frames)>900:raise ValueError('Frame count limit')
    choices=select_windows(frames);output.mkdir(parents=True,exist_ok=False);rows=[]
    for name,choice in choices.items():
        selected=frames[choice['start']:choice['end']];folder=output/name;folder.mkdir()
        variants={'baseline':selected,'interior':[]};counts={'valid_pixels':0,'edge_pixels':0,'spatially_removed':0}
        for record in selected:
            image,depth=read_depth(record);interior,edge,stats=depth_layers(depth)
            counts['valid_pixels']+=int((interior|edge).sum());counts['edge_pixels']+=int(edge.sum())
            counts['spatially_removed']+=stats['isolated_removed']
            variants['interior'].append(masked_record(record,image,interior))
        row=dict(name=name,selection=choice,frames=len(selected),counts=counts,previews={},reports={},
                 endpoint_consistency=compare_frames(selected[0],selected[-1]))
        for label,records in variants.items():
            mesh,supported,report=fuse_depth_records(records)
            path=folder/(label+'.ply')
            if len(mesh.triangles):
                o3d.io.write_triangle_mesh(str(path),mesh)
                descriptor=preview_mesh(path,folder,label);descriptor['file']=name+'/'+descriptor['file'];row['previews'][label]=descriptor
            counts_triangles,_,areas=mesh.cluster_connected_triangles() if len(mesh.triangles) else ([],[],[])
            # Small components may be real chair parts; these are descriptive,
            # never labels declaring noise or authorizing deletion.
            report['components']=len(areas)
            report['small_component_area_fraction']=float(sum(a for a in areas if a<.0004)/sum(areas)) if sum(areas)>0 else None
            row['reports'][label]=report
        rows.append(row)
    result={'source_sha256':source_hash,'cases':rows,'online_enabled':False,
            'warning':'Interior-only is an ablation: it also removes real edges and thin chair structure. No accuracy claim.',
            'limitations':['Different motion windows also have different viewpoints and occlusions; comparison is not a pure causal experiment.',
                           'Both meshes use original poses; no mesh smoothing, hole filling or pose optimization.']}
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    (output/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    install_viewer(output)
    return result


def install_viewer(output):
    output=Path(output)
    (output/'index.html').write_bytes(Path(__file__).with_name('noise_audit_viewer.html').read_bytes())
    three=Path(sys.executable).parents[2]/'web/node_modules/three'
    for name,source in [('three.module.js',three/'build/three.module.js'),
                        ('three.core.js',three/'build/three.core.js'),
                        ('OrbitControls.js',three/'examples/jsm/controls/OrbitControls.js')]:
        shutil.copyfile(source,output/name)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session');parser.add_argument('output')
    args=parser.parse_args();r=build(args.session,args.output)
    print(json.dumps([{'name':c['name'],'selection':c['selection'],'counts':c['counts'],
                      'median_endpoint_depth_m':c['endpoint_consistency']['absolute_depth_residual_m'].get('median'),
                      'reports':{k:{f:v for f,v in x.items() if f in ('all_observations','components','small_component_area_fraction')} for k,x in c['reports'].items()}} for c in r['cases']],indent=2))
