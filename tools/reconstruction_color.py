"""Bounded RGB-assisted local geometry. Requires independently verified calibration."""
import base64
import hashlib
import json
import zlib
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from reconstruction_core import decode_depth_frame,checked_pose
from reconstruction_segments import corrected_record
from reconstruction_tsdf import fuse_depth_records
from rgbd_projection_check import project
from benchmark_open3d_registration import assess_candidate


def calibration(path):
    c=json.loads(Path(path).read_text(encoding='utf-8'))
    if c.get('spatial_registration_verified') is not True or not c.get('verification_evidence'):
        raise ValueError('彩色/深度标定尚未验证，不能执行照片辅助几何重建')
    if c.get('translation_unit')!='meter' or c.get('model')!='brown_conrady':
        raise ValueError('需要明确的米制外参和Brown-Conrady彩色模型')
    t=np.asarray(c['depth_to_color'],float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
            or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3,:3]),1.)):
        raise ValueError('Invalid depth-to-color transform')
    for key,n in [('color_intrinsics',4),('distortion',5),('depth_intrinsics',4),('color_size',2)]:
        a=np.asarray(c[key],float)
        if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('Invalid '+key)
    if min(c['color_intrinsics'][:2])<=0:raise ValueError('Invalid focal length')
    return c


def photo(sample):
    w,h,step=(sample[k] for k in ('width','height','step'))
    if (any(type(x) is not int for x in (w,h,step)) or not 1<=w<=1280 or not 1<=h<=1024
            or not w*3<=step<=w*3+4096 or sample['encoding'] not in ('rgb8','bgr8')):
        raise ValueError('Invalid color layout')
    decoder=zlib.decompressobj();raw=decoder.decompress(base64.b64decode(sample['data_zlib_b64'],validate=True),step*h+1)
    if len(raw)!=step*h or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('Invalid color payload')
    rgb=np.ndarray((h,w,3),dtype=np.uint8,buffer=raw,strides=(step,3,1))
    return (rgb[:,:,::-1] if sample['encoding']=='bgr8' else rgb).copy()


def colored_cloud(frame,c):
    s=frame.get('color_sample')
    if not s or abs(s['stamp']-frame['stamp'])>.05 or abs(s['depth_stamp']-frame['stamp'])>1e-6:
        raise ValueError('Missing or mistimed color/depth pair')
    if frame['source_frame']!=c['depth_frame'] or not np.allclose(frame['intrinsics'],c['depth_intrinsics'],rtol=0,atol=1e-6):
        raise ValueError('Depth camera differs from verified calibration')
    rgb=photo(s);h,w=rgb.shape[:2]
    if [w,h]!=c['color_size']:raise ValueError('Color dimensions differ from calibration')
    local=dict(frame,translation=[0.,0.,0.],quaternion=[0.,0.,0.,1.])
    xyz=decode_depth_frame(local,stride=2);t=np.asarray(c['depth_to_color'])
    q=xyz@t[:3,:3].T+t[:3,3];xyz=xyz[q[:,2]>.05]
    uv,z=project(xyz,t[:3,:3],t[:3,3],c['color_intrinsics'],c['distortion'],True)
    ok=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<w-.5)&(uv[:,1]>=0)&(uv[:,1]<h-.5)
    xyz,uv,z=xyz[ok],np.rint(uv[ok]).astype(int),z[ok]
    index=uv[:,1]*w+uv[:,0];nearest=np.full(w*h,np.inf);np.minimum.at(nearest,index,z)
    visible=z<=nearest[index]+.01;xyz,uv=xyz[visible],uv[visible]
    pos,quat=checked_pose(frame);world=xyz@Rotation.from_quat(quat).as_matrix().T+pos
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(world))
    cloud.colors=o3d.utility.Vector3dVector(rgb[uv[:,1],uv[:,0]].astype(float)/255.)
    cloud=cloud.voxel_down_sample(.01)
    if len(cloud.points)<150:raise ValueError('Too few visible colored points')
    if np.std(np.asarray(cloud.colors))<.015:raise ValueError('Insufficient color variation')
    return cloud


def refine(source,target):
    # Hold every fifth source point out of optimization for geometry validation.
    points=np.asarray(source.points);fit=source.select_by_index(np.flatnonzero(np.arange(len(points))%5!=0))
    transform=np.eye(4)
    for voxel in (.04,.02,.01):
        a,b=fit.voxel_down_sample(voxel),target.voxel_down_sample(voxel)
        for cloud in (a,b):cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=30))
        result=o3d.pipelines.registration.registration_colored_icp(a,b,voxel*2,transform,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
        transform=result.transformation
    decision=assess_candidate(points,np.asarray(target.points),transform)
    if not decision['qualified']:raise ValueError('彩色配准未通过几何检查：'+decision['reason'])
    return (transform if decision['decision']=='apply_correction' else np.eye(4)),decision


def run_color(session,output,first_frame=1):
    from reconstruction_offline_worker import progress,preview_mesh
    session,output=Path(session),Path(output)
    progress(output,'calibration','正在验证照片和深度标定')
    cp=session/'rgbd_calibration.json'
    if not cp.is_file():raise ValueError('缺少本次采集已验证的rgbd_calibration.json；照片辅助建模尚不能启动')
    c=calibration(cp);archive=session/'frames.jsonl';sha=hashlib.sha256(archive.read_bytes()).hexdigest()
    if archive.stat().st_size>256*1024*1024:raise ValueError('Archive too large')
    rows=[json.loads(line) for line in archive.read_text(encoding='utf-8').splitlines()]
    if not 1<=first_frame<=900 or len(rows)>900:raise ValueError('Frame selection outside local bounds')
    selected=[(i+1,f) for i,f in enumerate(rows) if i+1>=first_frame and f.get('color_sample')][:10]
    if len(selected)<3:raise ValueError('至少需要3组带照片的深度记录；旧纯深度扫描无法补造照片')
    if len({(f['frame'],f['source_frame']) for _,f in selected})!=1:raise ValueError('Coordinate frame changed')
    if any(b['stamp']<=a['stamp'] for (_,a),(_,b) in zip(selected,selected[1:])):raise ValueError('Unordered frames')
    clouds=[colored_cloud(f,c) for _,f in selected];derived=[selected[0][1]];checks=[];target=clouds[0]
    progress(output,'alignment','正在联合颜色与几何配准局部照片序列')
    for (index,f),cloud in zip(selected[1:],clouds[1:]):
        transform,check=refine(cloud,target);checks.append(dict(frame=index,**check))
        derived.append(corrected_record(f,transform));target=cloud.transform(transform)
    (output/'color_alignment.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
    (output/'frames_color_corrected.jsonl').write_text('\n'.join(json.dumps(f) for f in derived)+'\n',encoding='utf-8')
    progress(output,'fusion','正在用颜色辅助位姿生成灰模')
    folder=output/'fusion';folder.mkdir();previews={}
    for name,records in [('observed',[f for _,f in selected]),('aligned',derived)]:
        mesh,_,report=fuse_depth_records(records)
        path=folder/(name+'_gray.ply');o3d.io.write_triangle_mesh(str(path),mesh)
        preview=preview_mesh(path,output,name)
        if not preview:raise ValueError('No surface extracted')
        previews[name]=preview
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=sha:raise ValueError('Source changed')
    result=dict(session=session.name,source_sha256=sha,frames=len(derived),color_assisted=True,
        calibration_sha256=hashlib.sha256(cp.read_bytes()).hexdigest(),
        selected_frames=[i for i,_ in selected],processed_points=0,excluded_points=0,
        observed_triangles=previews['observed']['full_triangles'],supported_triangles=0,
        previews=previews,absolute_accuracy_validated=False,local_only=True,
        note='仅选定彩色关键帧局部重建；非整段闭环，无缺失表面补造')
    (output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    progress(output,'done','照片辅助局部灰模完成；请对比原位姿与修正结果',results=result)
