"""Compare depth-range-limited fusion; original poses and archives stay intact."""
import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from reconstruction_analysis import compare_frames
from reconstruction_noise_audit import select_windows
from reconstruction_tsdf import fuse_depth_records
from reconstruction_offline_worker import preview_mesh


def build(session,baseline,output,max_depth=2.):
    session,baseline,output=map(Path,(session,baseline,output))
    if output.exists():raise FileExistsError('Use a new output directory')
    archive=session/'frames.jsonl'
    if not 0<archive.stat().st_size<=256*1024*1024:raise ValueError('Archive size limit')
    raw=archive.read_bytes();source_hash=hashlib.sha256(raw).hexdigest()
    records=[json.loads(line) for line in raw.splitlines()]
    if not 10<=len(records)<=900:raise ValueError('Expected 10 to 900 frames')
    prior=json.loads((baseline/'result.json').read_text(encoding='utf-8'))
    if prior['source_sha256']!=source_hash:raise ValueError('Baseline capture does not match')
    windows=select_windows(records)
    # All adjacent pairs in the preselected static window, in both directions.
    # Medians are reported per pair; they are not pooled metrology samples.
    static=records[windows['static']['start']:windows['static']['end']]
    comparisons=[]
    for a,b in zip(static,static[1:]):
        for ref,src in ((a,b),(b,a)):comparisons.append(compare_frames(ref,src))
    bands={}
    for key in comparisons[0]['residual_by_projected_depth']:
        values=[r['residual_by_projected_depth'][key]['median'] for r in comparisons
                if r['residual_by_projected_depth'][key]['count']]
        bands[key]={'pairs_with_samples':len(values),
                    'median_of_pair_medians_m':float(np.median(values)) if values else None}
    output.mkdir(parents=True,exist_ok=False)
    mesh,supported,fusion=fuse_depth_records(records,max_depth=max_depth)
    previews={}
    for name,model in (('near',mesh),('supported',supported)):
        if len(model.triangles):
            path=output/(name+'.ply')
            if not o3d.io.write_triangle_mesh(str(path),model):raise OSError('Mesh export failed')
            previews[name]=preview_mesh(path,output,name)
    # Reuse the existing baseline preview without altering its full PLY.
    descriptor=dict(prior['previews']['observed'])
    shutil.copyfile(baseline/descriptor['file'],output/'baseline.bin')
    descriptor['file']='baseline.bin';previews['baseline']=descriptor
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    result={'source_sha256':source_hash,'source_unchanged':True,'frames':len(records),
            'max_depth_m':max_depth,'previews':previews,'static_window':windows['static'],
            'static_depth_bands':bands,'static_pair_checks':comparisons,'fusion':fusion,
            'pose_correction':False,'absolute_accuracy_validated':False,'online_enabled':False,
            'limitations':['Range is measured from each camera pose, not distance from the first pose.',
                          'Reduced coverage is intentional; excluded far depth remains in the raw archive.',
                          'Reprojection residuals include noise, occlusion, pose and sampling effects.',
                          'No synthesized chair seat, hole filling, smoothing or claimed loop closure.']}
    (output/'range_report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'index.html').write_bytes(Path(__file__).with_name('range_compare_viewer.html').read_bytes())
    three=Path(sys.executable).parents[2]/'web/node_modules/three'
    for name,source in [('three.module.js',three/'build/three.module.js'),
                        ('three.core.js',three/'build/three.core.js'),
                        ('OrbitControls.js',three/'examples/jsm/controls/OrbitControls.js')]:
        shutil.copyfile(source,output/name)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session');p.add_argument('baseline');p.add_argument('output')
    p.add_argument('--max-depth',type=float,default=2.)
    args=p.parse_args();r=build(args.session,args.baseline,args.output,args.max_depth)
    print(json.dumps({k:r[k] for k in ('frames','source_unchanged','max_depth_m','static_depth_bands')},indent=2))
