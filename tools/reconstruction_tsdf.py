"""Offline depth-data fusion via Open3D Tensor TSDF; no mesh polishing or motion."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import open3d as o3d
from reconstruction_core import checked_pose
from reconstruction_mesh import depth_and_camera
from reconstruction_postprocess import filter_depth


def fuse_depth_records(records,voxel=.02,max_blocks=4000):
    records=list(records)
    if not 1<=len(records)<=900:raise ValueError('Expected 1 to 900 depth records')
    if not np.isfinite(voxel) or not .01<=voxel<=.05:raise ValueError('Voxel must be 1 to 5 cm')
    if type(max_blocks) is not int or not 1<=max_blocks<=8000:raise ValueError('Invalid block limit')
    previous=0.;frame=None;anchor=None
    for record in records:
        t,_=checked_pose(record)
        origin=(record['frame'],record['source_frame'])
        if frame is not None and frame!=origin:raise ValueError('Coordinate frame changed')
        if record['stamp']<=previous:raise ValueError('Non-increasing timestamps')
        if anchor is None:anchor=t
        if np.linalg.norm(t-anchor)>5.:raise ValueError('Offline local fusion exceeds 5m camera radius')
        frame,previous=origin,record['stamp']
    volume=o3d.t.geometry.VoxelBlockGrid(attr_names=('tsdf','weight'),
        attr_dtypes=(o3d.core.float32,o3d.core.float32),attr_channels=((1,),(1,)),
        voxel_size=voxel,block_resolution=16,block_count=max_blocks,device=o3d.core.Device('CPU:0'))
    active=set();rows=[]
    for index,record in enumerate(records):
        depth,intrinsic,extrinsic=depth_and_camera(record)
        filtered,stats=filter_depth(depth)
        if not stats['retained_pixels']:
            rows.append(dict(stats,frame=index+1,active_blocks=len(active)))
            continue
        image=o3d.t.geometry.Image(o3d.core.Tensor(filtered.astype(np.float32)))
        k=o3d.core.Tensor(np.array(intrinsic.intrinsic_matrix),dtype=o3d.core.float64)
        e=o3d.core.Tensor(extrinsic,dtype=o3d.core.float64)
        coordinates=volume.compute_unique_block_coordinates(image,k,e,1.,4.,4.)
        additions=set(map(tuple,coordinates.numpy().tolist()))
        if len(active|additions)>max_blocks:raise ValueError('Offline TSDF block limit reached; no blocks silently dropped')
        volume.integrate(coordinates,image,k,e,1.,4.,4.)
        active.update(additions)
        rows.append(dict(stats,frame=index+1,active_blocks=len(active)))
    meshes=[]
    for threshold in (.5,2.5):
        mesh=o3d.geometry.TriangleMesh()
        if active and len(records)>threshold:
            try:
                mesh=volume.extract_triangle_mesh(weight_threshold=threshold).to_legacy()
            except RuntimeError as error:
                # This build raises instead of returning an empty mesh when
                # the requested confidence layer has no extractable surface.
                if 'SetVertexColors' not in str(error) or 'Tensor has shape {0}' not in str(error):raise
        mesh.remove_degenerate_triangles();mesh.compute_vertex_normals()
        if not np.isfinite(np.asarray(mesh.vertices)).all():raise ValueError('Extracted mesh is not finite')
        mesh.paint_uniform_color([.65,.65,.65]);meshes.append(mesh)
    report={'frames':len(records),'method':'Open3D Tensor VoxelBlockGrid projective TSDF',
            'open3d':o3d.__version__,'device':'CPU:0','frame':frame[0],
            'voxel_m':voxel,'truncation_m':voxel*4,'active_blocks':len(active),'block_limit':max_blocks,
            'all_observations':{'weight_threshold':.5,'vertices':len(meshes[0].vertices),'triangles':len(meshes[0].triangles)},
            'higher_support':{'weight_threshold':2.5,'vertices':len(meshes[1].vertices),'triangles':len(meshes[1].triangles)},
            'depth_spatial_outliers_removed':sum(r['isolated_removed'] for r in rows),
            'integration_rows':rows,'pose_correction':False,'mesh_smoothing':False,'color_integration':False,
            'absolute_accuracy_validated':False,'loop_closure':False,'control_requests':0,
            'limitations':['Uses unchanged original timestamp poses; misalignment can remain.',
                          'TSDF averaging is data fusion, and can blur thin structures at the chosen voxel size.',
                          'Support thresholds are integration weights, not independent accuracy or semantic validation.',
                          'No deliberate hole filling; interpolation is still inherent to surface extraction.',
                          'Only recorded local coverage, not a complete room or watertight model.']}
    return meshes[0],meshes[1],report


def build(session,output):
    session,output=Path(session),Path(output)
    if output.exists():raise FileExistsError('Use a new output directory')
    archive=session/'frames.jsonl'
    if not archive.is_file() or archive.stat().st_size>256*1024*1024:raise ValueError('Expected bounded raw frame archive')
    source_hash=hashlib.sha256(archive.read_bytes()).hexdigest()
    records=[]
    with archive.open(encoding='utf-8') as stream:
        for line in stream:
            records.append(json.loads(line))
            if len(records)>900:raise ValueError('Too many frames for offline trial')
    start=time.perf_counter()
    all_mesh,supported,report=fuse_depth_records(records)
    if not len(all_mesh.triangles):raise ValueError('No observed surface extracted')
    output.mkdir(parents=True,exist_ok=False)
    for name,mesh in [('observed_gray',all_mesh),('supported_gray',supported)]:
        if len(mesh.triangles) and not o3d.io.write_triangle_mesh(str(output/(name+'.ply')),mesh):
            raise OSError('Mesh export failed')
    original=json.loads((session/'summary.json').read_text(encoding='utf-8'))
    info=original.get('raw_archive') or {}
    report.update(source=str(archive.resolve()),source_sha256=source_hash,
                  source_capture_termination=original.get('error'),
                  all_accepted_frames_used=bool(info.get('complete') and len(records)==info.get('frames')==original.get('accepted_frames')),
                  processing_seconds=time.perf_counter()-start)
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed during fusion')
    (output/'fusion_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=build(args.session,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='integration_rows'},ensure_ascii=False,indent=2))
