"""Experimental offline multiscale candidate; held-out acceptance stays separate."""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from benchmark_open3d_registration import assess_candidate
from reconstruction_registration import sampled_cloud
from reconstruction_registration import apply_transform


def track_step(cloud,prior,references,solver=None):
    """Return no updated pose if local or cumulative validation fails."""
    solver=solver or propose
    quality=solver(apply_transform(cloud,prior),np.concatenate(references[-3:]))
    if not quality['qualified']:return quality,None
    combined=np.array(quality['transform'])@prior
    angle=float(np.degrees(Rotation.from_matrix(combined[:3,:3]).magnitude()))
    center=cloud.mean(axis=0)
    shift=float(np.linalg.norm(apply_transform(center[None],combined)[0]-center))
    quality=dict(quality,cumulative_angle_deg=angle,cumulative_shift_m=shift)
    if angle>5 or shift>.05:
        return dict(quality,qualified=False,decision='reject',reason='cumulative_correction_limit'),None
    return quality,combined


def split_tracks(clouds,solver=None):
    """Partition ALL views once, restarting from odometry at each unsafe edge.

    Segments have independent correction anchors and MUST NOT be silently fused
    as one corrected room. Even a one-frame segment is retained for review.
    """
    if not clouds:raise ValueError('No clouds')
    groups=[{'indices':[0],'corrections':[np.eye(4)],'start_reason':'first_frame','checks':[]}]
    prior=np.eye(4);references=[sampled_cloud(clouds[0])]
    for index in range(1,len(clouds)):
        quality,combined=track_step(clouds[index],prior,references,solver)
        if combined is None:
            groups.append({'indices':[index],'corrections':[np.eye(4)],'start_reason':quality['reason'],'checks':[]})
            groups[-1]['rejected_incoming_edge']=quality
            prior=np.eye(4);references=[sampled_cloud(clouds[index])]
        else:
            groups[-1]['indices'].append(index);groups[-1]['corrections'].append(combined)
            groups[-1]['checks'].append(quality);prior=combined
            references.append(sampled_cloud(apply_transform(clouds[index],prior)));references=references[-3:]
    return groups


def propose(source,target):
    source,target=sampled_cloud(source),sampled_cloud(target)
    if min(len(source),len(target))<100:
        return {'qualified':False,'decision':'reject','reason':'too_few_points','transform':np.eye(4).tolist()}
    # Do not let validation points enter ANY scale of the optimizer.
    training=source[np.arange(len(source))%5!=0]
    src,dst=o3d.geometry.PointCloud(),o3d.geometry.PointCloud()
    src.points=o3d.utility.Vector3dVector(training)
    dst.points=o3d.utility.Vector3dVector(target)
    transform=np.eye(4)
    reg=o3d.pipelines.registration
    for voxel,distance,iterations in ((.06,.12,35),(.03,.08,25),(.02,.05,20)):
        a,b=src.voxel_down_sample(voxel),dst.voxel_down_sample(voxel)
        if min(len(a.points),len(b.points))<30:
            return {'qualified':False,'decision':'reject','reason':'too_few_scale_points','transform':np.eye(4).tolist()}
        b.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=30))
        result=reg.registration_icp(a,b,distance,transform,
            reg.TransformationEstimationPointToPlane(reg.HuberLoss(.01)),
            reg.ICPConvergenceCriteria(max_iteration=iterations))
        transform=result.transformation
    assessment=assess_candidate(source,target,transform)
    # Rejected/unchanged candidates never carry an applied correction.
    applied=transform if assessment['decision']=='apply_correction' and assessment['qualified'] else np.eye(4)
    return dict(assessment,transform=applied.tolist(),proposed_transform=transform.tolist())
