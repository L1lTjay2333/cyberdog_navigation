"""Bounded depth-only pose graph and first-order MLS, PC offline only."""
import hashlib
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from reconstruction_registration import apply_transform
from reconstruction_multiscale import propose
from benchmark_open3d_registration import assess_candidate
from reconstruction_turn_audit import load_views
from reconstruction_segments import corrected_record
from reconstruction_tsdf import fuse_depth_records


def validate_joint_step(clouds,edges,corrections):
    """Backtrack an over-large joint update; keep identical acceptance limits.

    These are validation samples reused for step selection, not an independent
    test set or accuracy certificate. Zero step is never called an improvement.
    """
    trials=[]
    for scale in (1.,.5,.25,.125):
        candidate=[]
        for c in corrections:
            damped=np.eye(4)
            damped[:3,:3]=Rotation.from_rotvec(Rotation.from_matrix(c[:3,:3]).as_rotvec()*scale).as_matrix()
            damped[:3,3]=c[:3,3]*scale;candidate.append(damped)
        bounded=all(np.linalg.norm(apply_transform(cloud.mean(axis=0)[None],c)[0]-cloud.mean(axis=0))<=.05
                    and np.degrees(Rotation.from_matrix(c[:3,:3]).magnitude())<=5
                    for cloud,c in zip(clouds,candidate))
        if not bounded:
            trials.append({'scale':scale,'validation':[],'reason':'joint_correction_limit'})
            continue
        checks=[]
        for i,j,_ in edges:
            q=assess_candidate(clouds[i],clouds[j],np.linalg.inv(candidate[j])@candidate[i])
            checks.append(dict(source=i,target=j,**q))
        trials.append({'scale':scale,'validation':checks})
        if all(q['qualified'] for q in checks):return candidate,trials
    return None,trials


def joint_align(clouds):
    """All nodes must connect through qualified edges; reject unsupported output."""
    reg=o3d.pipelines.registration;n=len(clouds)
    if not 3<=n<=10:raise ValueError('Expected 3 to 10 local views')
    edges=[];checks=[]
    for i in range(n):
        for j in range(i):
            q=propose(clouds[i],clouds[j]);checks.append(dict(source=i,target=j,**q))
            if q['qualified']:edges.append((i,j,np.array(q['transform'])))
    poses={0:np.eye(4)}
    while True:
        old=len(poses)
        for i,j,t in edges:
            if j in poses and i not in poses:poses[i]=poses[j]@t
            elif i in poses and j not in poses:poses[j]=poses[i]@np.linalg.inv(t)
        if len(poses)==old:break
    report={'pair_checks':checks,'qualified_edges':len(edges),'frames':n,'accepted':False,
            'absolute_accuracy_validated':False,'loop_closure':False}
    if len(poses)!=n:return None,dict(report,reason='disconnected_local_graph')
    graph=reg.PoseGraph()
    for i in range(n):graph.nodes.append(reg.PoseGraphNode(poses[i]))
    for i,j,t in edges:
        # All geometric edges remain uncertain, including temporal neighbors.
        a=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(clouds[i]))
        b=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(clouds[j]))
        info=reg.get_information_matrix_from_point_clouds(a,b,.04,t)
        graph.edges.append(reg.PoseGraphEdge(i,j,t,info,uncertain=True))
    reg.global_optimization(graph,reg.GlobalOptimizationLevenbergMarquardt(),
        reg.GlobalOptimizationConvergenceCriteria(),
        reg.GlobalOptimizationOption(max_correspondence_distance=.04,edge_prune_threshold=.25,reference_node=0))
    corrections=[np.array(node.pose) for node in graph.nodes];validation=[]
    corrections,trials=validate_joint_step(clouds,edges,corrections)
    if corrections is None:return None,dict(report,reason='joint_edge_validation_failed',step_trials=trials)
    validation=trials[-1]['validation']
    return corrections,dict(report,accepted=True,reason='qualified_local_graph',validation=validation,
                             joint_step_scale=trials[-1]['scale'],step_trials=trials,
                             corrections=[c.tolist() for c in corrections])


def mls_project(points,radius=.03,max_shift=.005,normals=None):
    """First-order weighted local-plane MLS. No upsampling or hole filling.

    Line-like neighborhoods, corners and sparse patches are left untouched.
    This is our bounded implementation of the MLS idea, not a PCL binding.
    """
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or len(p)>200000:
        raise ValueError('Expected bounded finite points')
    if not 0<max_shift<radius<=.05:raise ValueError('Invalid MLS scale')
    if normals is not None:
        normals=np.asarray(normals,dtype=float)
        if normals.shape!=p.shape or not np.isfinite(normals).all():raise ValueError('Invalid normals')
        lengths=np.linalg.norm(normals,axis=1)
        usable=lengths>1e-8
        normals=normals/np.maximum(lengths[:,None],1e-8)
    output=p.copy();tree=cKDTree(p);moved=0
    for i,point in enumerate(p):
        ids=tree.query_ball_point(point,radius)
        if normals is not None:
            if not usable[i]:continue
            # Signed normals distinguish nearby front/back surfaces. Sharp
            # bends and different faces must not vote for the same plane.
            ids=[j for j in ids if usable[j] and normals[j]@normals[i]>=np.cos(np.pi/6)]
        if len(ids)<12:continue
        local=p[ids];w=np.exp(-np.sum((local-point)**2,axis=1)/(radius*.5)**2)
        center=np.average(local,axis=0,weights=w);d=local-center
        values,vectors=np.linalg.eigh((d*w[:,None]).T@d/w.sum())
        if values[1]<1e-8 or values[0]>.08*values[1] or values[1]<.2*values[2]:continue
        normal=vectors[:,0];shift=np.dot(center-point,normal)
        if abs(shift)<=max_shift:output[i]+=shift*normal;moved+=abs(shift)>1e-9
    return output,{'method':'bounded first-order weighted local-plane MLS','radius_m':radius,
                   'max_shift_m':max_shift,'moved_points':int(moved),'points':len(p),
                   'normal_gate_deg':30 if normals is not None else None,
                   'upsampling':False,'hole_filling':False}


def run_local(session,output,start=1):
    from reconstruction_offline_worker import progress,preview_mesh
    frames,clouds,_,source_hash=load_views(session,max_depth=2.)
    if type(start) is not int or start<1 or start+9>len(frames):raise ValueError('请选择有连续10帧的起始帧')
    frames=frames[start-1:start+9];clouds=clouds[start-1:start+9]
    progress(output,'alignment','正在检查10个视角并联合对齐')
    corrections,alignment=joint_align(clouds)
    (output/'alignment.json').write_text(json.dumps(alignment,ensure_ascii=False,indent=2),encoding='utf-8')
    accepted=corrections is not None
    # Still provide an honest point-surface comparison when alignment fails.
    # Do not publish the rejected joint poses as an aligned model.
    if corrections is None:corrections=[np.eye(4) for _ in frames]
    derived=[corrected_record(f,c) for f,c in zip(frames,corrections)]
    # Reprojection uses the original depth images, rather than the solver's
    # subsampled point-to-plane objective. It still includes occlusion effects.
    from reconstruction_analysis import compare_frames
    depth_checks=[]
    for a,b in ((0,4),(4,9),(0,9)):
        for i,j in ((a,b),(b,a)):
            depth_checks.append({'reference_frame':start+i,'source_frame':start+j,
                'original':compare_frames(frames[i],frames[j]),
                'corrected':compare_frames(derived[i],derived[j])})
    (output/'reprojection.json').write_text(json.dumps({'pairs':depth_checks,
        'absolute_accuracy_validated':False,'note':'Depth-domain diagnostic, includes occlusion and viewpoint effects.'},indent=2),encoding='utf-8')
    with (output/'frames_corrected.jsonl').open('x',encoding='utf-8') as stream:
        for f in derived:stream.write(json.dumps(f,separators=(',',':'))+'\n')
    previews={};folder=output/'fusion';folder.mkdir()
    progress(output,'fusion','正在生成相同分辨率的原位姿与联合对齐灰模')
    for key,records in [('observed',frames),('aligned',derived)]:
        mesh,supported,report=fuse_depth_records(records,voxel=.01,max_depth=2.)
        path=folder/(key+'_gray.ply');o3d.io.write_triangle_mesh(str(path),mesh)
        if key!='aligned' or accepted:previews[key]=preview_mesh(path,output,key)
        if key=='aligned':
            o3d.io.write_triangle_mesh(str(folder/'supported_gray.ply'),supported)
            previews['supported']=preview_mesh(folder/'supported_gray.ply',output,'supported')
            report.update(pose_correction=accepted,pose_refinement_enabled=accepted,local_only=True,
                          limitations=['Local ten-frame result; alignment acceptance is reported separately.',
                                       'No absolute accuracy validation or missing-surface completion.'])
            (folder/'fusion_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            aligned_mesh=mesh
    progress(output,'mls','正在处理点云局部表面并重建网格')
    # Mesh vertices retain TSDF's surface normals for ball pivoting. MLS changes
    # point positions before a NEW mesh is built; it does not smooth mesh faces.
    points,mls=mls_project(np.asarray(aligned_mesh.vertices),normals=np.asarray(aligned_mesh.vertex_normals))
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.normals=o3d.utility.Vector3dVector(np.asarray(aligned_mesh.vertex_normals))
    cloud.normalize_normals()
    model=o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud,o3d.utility.DoubleVector([.015,.025]))
    model.remove_degenerate_triangles();model.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(output/'mls.ply'),model)
    previews['mls']=preview_mesh(output/'mls.ply',output,'mls')
    processed=output/'processed';processed.mkdir()
    shown=points[np.linspace(0,len(points)-1,min(30000,len(points))).astype(int)].astype('<f4')
    shown.tofile(output/'points.bin')
    if len(points):previews['points']={'file':'points.bin','kind':'points','displayed_points':len(shown),
        'full_points':len(points),'position_count':len(shown),'minimum':points.min(axis=0).tolist(),
        'maximum':points.max(axis=0).tolist()}
    o3d.io.write_point_cloud(str(processed/'points_processed.ply'),cloud)
    (processed/'processing_report.json').write_text(json.dumps(mls,indent=2),encoding='utf-8')
    if hashlib.sha256((Path(session)/'frames.jsonl').read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    result={'session':Path(session).name,'source_sha256':source_hash,'frames':10,'first_frame':start,
            'local_refinement':True,'pose_correction':accepted,'alignment_accepted':accepted,
            'alignment_reason':alignment['reason'],'absolute_accuracy_validated':False,
            'processed_points':len(points),'excluded_points':0,
            'observed_triangles':previews['observed']['full_triangles'],
            'supported_triangles':previews['supported']['full_triangles'] if previews['supported'] else 0,
            'previews':{k:v for k,v in previews.items() if v},
            'note':('局部对齐通过；' if accepted else '联合对齐未通过，曲面处理使用原位姿；')+
                   '仅10帧；曲面版本同时包含MLS和球旋转重建，不能将差异全部归因于MLS。没有补全缺失座面。'}
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    progress(output,'done','10帧局部处理完成；请比较结构、厚度与缺失区域',results=result)
