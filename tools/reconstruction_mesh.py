"""Offline depth-grid surface experiment. No robot/network access."""
import argparse
import base64
import json
from pathlib import Path
import time
import zlib
import numpy as np
import open3d as o3d
from reconstruction_core import checked_pose,decode_depth_frame
from reconstruction_analysis import rotation


def depth_and_camera(record):
    # Reuse bounded payload, layout, finite calibration and pose validation.
    decode_depth_frame(record)
    item=2 if record['encoding']=='16UC1' else 4
    dtype=('>' if record['bigendian'] else '<')+('u2' if item==2 else 'f4')
    raw=zlib.decompress(base64.b64decode(record['depth_zlib_b64'],validate=True))
    depth=np.ndarray((record['height'],record['width']),dtype=dtype,buffer=raw,
                     strides=(record['step'],item)).astype(np.float32)
    if item==2:depth*=.001
    depth[~np.isfinite(depth)|(depth<.15)|(depth>4.)]=0.
    fx,fy,cx,cy=record['intrinsics']
    intrinsic=o3d.camera.PinholeCameraIntrinsic(record['width'],record['height'],fx,fy,cx,cy)
    t,_=checked_pose(record);r=rotation(record)
    # Saved pose maps camera -> world; Open3D integrate expects world -> camera.
    extrinsic=np.eye(4);extrinsic[:3,:3]=r.T;extrinsic[:3,3]=-r.T.dot(t)
    return np.ascontiguousarray(depth),intrinsic,extrinsic


def integrate_records(records):
    records=list(records)
    if not 1<=len(records)<=900:raise ValueError('Expected 1 to 900 frames')
    previous=0.;origin=None;anchor=None
    # Validate all poses before allocating a volume; experiment has spatial bounds.
    for record in records:
        t,_=checked_pose(record)
        current=(record['frame'],record['source_frame'])
        if origin is not None and current!=origin:raise ValueError('Coordinate frame changed')
        if record['stamp']<=previous:raise ValueError('Non-increasing timestamps')
        if anchor is None:anchor=t
        if np.linalg.norm(t-anchor)>5.:raise ValueError('Offline local mesh trial exceeds 5m camera radius')
        origin,previous=current,record['stamp']
    vertices=[];triangles=[];offset=0
    for record in records:
        depth,intrinsic,extrinsic=depth_and_camera(record)
        # Organized depth cells create observed surfaces without filling holes.
        stride=4
        z=depth[::stride,::stride]
        v,u=np.mgrid[0:record['height']:stride,0:record['width']:stride]
        fx,fy,cx,cy=record['intrinsics']
        points=np.column_stack(((u.ravel()-cx)*z.ravel()/fx,
                               (v.ravel()-cy)*z.ravel()/fy,z.ravel()))
        r=rotation(record);t,_=checked_pose(record)
        points=points.dot(r.T)+t
        h,w=z.shape;grid=np.arange(h*w).reshape(h,w)
        a=grid[:-1,:-1].ravel();b=grid[:-1,1:].ravel()
        c=grid[1:,:-1].ravel();d=grid[1:,1:].ravel()
        faces=np.concatenate((np.column_stack((a,b,c)),np.column_stack((b,d,c))))
        depths=z.ravel()[faces]
        valid=(depths>0).all(axis=1)&(np.ptp(depths,axis=1)<=.04)
        faces=faces[valid]
        if len(faces):
            edges=points[faces]
            lengths=np.linalg.norm(edges-np.roll(edges,1,axis=1),axis=2)
            faces=faces[(lengths<=.10).all(axis=1)]
        if not len(faces):continue
        used,inverse=np.unique(faces,return_inverse=True)
        vertices.append(points[used]);triangles.append(inverse.reshape(-1,3)+offset)
        offset+=len(used)
    mesh=o3d.geometry.TriangleMesh()
    if vertices:
        mesh.vertices=o3d.utility.Vector3dVector(np.concatenate(vertices))
        mesh.triangles=o3d.utility.Vector3iVector(np.concatenate(triangles))
    mesh.remove_duplicated_vertices();mesh.remove_duplicated_triangles();mesh.remove_degenerate_triangles()
    mesh.compute_vertex_normals()
    vertices=np.asarray(mesh.vertices)
    if not np.isfinite(vertices).all():raise ValueError('Non-finite extracted mesh')
    report={'integrated_frames':len(records),'frame':origin[0],'method':'organized_depth_triangles','pixel_stride':4,'max_depth_edge_m':.04,'max_edge_m':.10,
            'vertices':len(mesh.vertices),'triangles':len(mesh.triangles),
            'bounds_m':[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()] if len(vertices) else None,
            'pose_source':'uncorrected original timestamp vodom','color_integrated':False,
            'absolute_accuracy_validated':False,'loop_closure':False,'control_requests':0,
            'limitations':['No new pose correction or loop closure; input pose errors remain in the surface.',
                          'Only observed local surfaces; not a complete room or watertight model.',
                          'No cross-frame surface fusion; overlapping surfaces may remain.',
                          'Depth discontinuities and moving objects are not separately segmented.']}
    return mesh,report


def build_mesh(session,output):
    session,output=Path(session),Path(output)
    archive=session/'frames.jsonl'
    if not archive.is_file() or archive.stat().st_size>256*1024*1024:
        raise ValueError('A size-bounded original frame archive is required')
    records=[]
    with archive.open(encoding='utf-8') as stream:
        for line in stream:
            records.append(json.loads(line))
            if len(records)>900:raise ValueError('Too many frames for offline mesh trial')
    started=time.perf_counter()
    mesh,report=integrate_records(records)
    if not len(mesh.triangles):raise ValueError('No surface extracted; original input preserved')
    output.mkdir(parents=True,exist_ok=False)
    # Geometry-only OBJ, with no MTL/textures. PLY carries a uniform neutral gray.
    mesh.paint_uniform_color([.65,.65,.65])
    if not o3d.io.write_triangle_mesh(str(output/'model_gray.ply'),mesh,write_ascii=False):raise OSError('PLY write failed')
    if not o3d.io.write_triangle_mesh(str(output/'model_gray.obj'),mesh,write_vertex_colors=False,
                                     write_triangle_uvs=False):raise OSError('OBJ write failed')
    summary=json.loads((session/'summary.json').read_text(encoding='utf-8'))
    info=summary.get('raw_archive') or {}
    report.update(session=str(session.resolve()),processing_seconds=time.perf_counter()-started,
        source_capture_termination=summary.get('error'),
        all_accepted_frames_used=bool(info.get('complete') and len(records)==summary.get('accepted_frames')==info.get('frames')),
        display='uniform gray, no textures or material file')
    (output/'mesh_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    print(json.dumps(build_mesh(args.session,args.output),ensure_ascii=False,indent=2))
