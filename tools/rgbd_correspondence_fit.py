"""Offline extrinsic diagnostic using measured 3D/2D landmarks, not hole edges.

Fit whole groups; validate independent withheld groups. Never overwrites stock
calibration or declares registration verified from one small fit residual.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def project_points(points, transform, intrinsics, distortion):
    q=np.asarray(points)@transform[:3,:3].T+transform[:3,3]
    if not np.isfinite(q).all() or np.any(q[:,2]<=.05):
        raise ValueError('Landmark behind color camera')
    x,y=(q[:,:2]/q[:,2,None]).T;k1,k2,p1,p2,k3=distortion
    r=x*x+y*y;radial=1+k1*r+k2*r*r+k3*r*r*r
    fx,fy,cx,cy=intrinsics
    return np.column_stack((fx*(x*radial+2*p1*x*y+p2*(r+2*x*x))+cx,
                            fy*(y*radial+2*p2*x*y+p1*(r+2*y*y))+cy))


def fit(data):
    points=np.asarray(data['depth_xyz_m'],float);uv=np.asarray(data['color_uv'],float)
    groups=data['groups'];validation=set(data['validation_groups'])
    if (points.ndim!=2 or points.shape[1]!=3 or uv.shape!=(len(points),2)
            or not 24<=len(points)<=500 or len(groups)!=len(points)
            or not np.isfinite(points).all() or not np.isfinite(uv).all()):
        raise ValueError('Need 24..500 finite measured 3D/2D correspondences')
    if not data.get('measurement_evidence'):
        raise ValueError('Measured correspondence evidence required')
    if len(set(groups))<3 or not validation or not validation<set(groups):
        raise ValueError('Need at least 3 groups and a separate withheld group')
    held=np.array([g in validation for g in groups]);train=~held
    if held.sum()<8 or train.sum()<16:raise ValueError('Need 16 fit and 8 withheld landmarks')
    if np.min(np.linalg.svd(points[train]-points[train].mean(axis=0),compute_uv=False))<.02:
        raise ValueError('Need spatially diverse landmarks; one flat board pose is insufficient')
    k=np.asarray(data['color_intrinsics'],float);d=np.asarray(data['distortion'],float)
    initial=np.asarray(data['initial_depth_to_color'],float)
    if (k.shape!=(4,) or d.shape!=(5,) or not np.isfinite(k).all() or not np.isfinite(d).all()
            or min(k[:2])<=0 or initial.shape!=(4,4) or not np.isfinite(initial).all()
            or not np.allclose(initial[3],[0,0,0,1])
            or not np.allclose(initial[:3,:3].T@initial[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(initial[:3,:3]),1)):
        raise ValueError('Invalid fixed camera model')
    def matrix(v):
        delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix();delta[:3,3]=v[3:]
        return delta@initial
    # Conservative residual correction only. Intrinsics are NOT jointly fitted.
    bounds=np.array([np.deg2rad(5)]*3+[.05]*3)
    def residual(v):
        try:return (project_points(points[train],matrix(v),k,d)-uv[train]).ravel()
        except ValueError:return np.full(int(train.sum())*2,1e6)
    solution=least_squares(residual,np.zeros(6),bounds=(-bounds,bounds),loss='soft_l1',f_scale=2.)
    candidate=matrix(solution.x)
    report={'measurement_evidence':data['measurement_evidence'],'fit_groups':sorted(set(groups)-validation),
            'validation_groups':sorted(validation),'fit_points':int(train.sum()),'validation_points':int(held.sum()),
            'candidate_depth_to_color':candidate.tolist(),'solver_success':bool(solution.success),
            'spatial_registration_verified':False,'automatic_calibration_export':False,
            'note':'Independent groups check transfer. Thresholds are diagnostic, not metrology certification.'}
    for name,t in [('initial',initial),('candidate',candidate)]:
        errors=np.linalg.norm(project_points(points,t,k,d)-uv,axis=1)
        report[name]={label:{'median_px':float(np.median(errors[mask])),
                                  'p95_px':float(np.percentile(errors[mask],95))}
                       for label,mask in [('fit',train),('validation',held)]}
    report['withheld_screen_pass']=bool(solution.success and report['candidate']['validation']['p95_px']<=3
        and report['candidate']['validation']['median_px']<=report['initial']['validation']['median_px'])
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('landmarks',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();data=json.loads(args.landmarks.read_text(encoding='utf-8'))
    result=fit(data)
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))
