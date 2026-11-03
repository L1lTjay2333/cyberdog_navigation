"""Central-ROI static planar depth diagnostics; not absolute range calibration.

Only use when the selected ROI observes one flat target.
No source data is modified. No RGB-depth registration is inferred.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def plane_residual(xyz):
    center=np.mean(xyz,axis=0)
    _,_,vt=np.linalg.svd(xyz-center,full_matrices=False)
    normal=vt[-1]
    residual=(xyz-center)@normal
    return center,normal,residual


def run(folder,roi=(.25,.25,.75,.60)):
    report=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    rows=[];images=[]
    for item in report['samples']:
        if 'depth_file' not in item:continue
        z=np.load(folder/item['depth_file']).astype(float)*(.001 if item['encoding']=='16UC1' else 1)
        h,w=z.shape
        x0,y0,x1,y1=roi
        if not 0<=x0<x1<=1 or not 0<=y0<y1<=1:raise ValueError('Invalid normalized ROI')
        v,u=np.mgrid[int(h*y0):int(h*y1):2,int(w*x0):int(w*x1):2]
        d=z[v,u];valid=np.isfinite(d)&(d>=.15)&(d<=4)
        infos=[c for c in item['calibration'] if c['frame_id']==item['frame_id']
               and c['width']==w and c['height']==h and abs(c['stamp']-item['stamp'])<=.2]
        if len(infos)!=1:raise ValueError('Missing unique recent calibration')
        p=infos[0]['p']
        if p[0]<=0 or p[5]<=0 or p[3]!=0 or p[7]!=0:raise ValueError('Unsupported depth projection')
        xyz=np.stack(((u-p[2])*d/p[0],(v-p[6])*d/p[5],d),axis=-1)[valid]
        if len(xyz)<1000:raise ValueError('Insufficient central valid points')
        center,normal,residual=plane_residual(xyz)
        abs_residual=np.abs(residual)
        rows.append({'pair_id':item.get('pair_id'),'roi_valid_fraction':float(valid.mean()),
            'roi_median_optical_z_m':float(np.median(d[valid])),
            'plane_distance_from_camera_m':float(abs(center@normal)),
            'plane_residual_rms_mm':float(np.sqrt(np.mean(residual**2))*1000),
            'plane_residual_p95_mm':float(np.percentile(abs_residual,95)*1000)})
        images.append(np.where(valid,d,np.nan))
    if len(images)<2:raise ValueError('At least two static samples required')
    stack=np.stack(images);common=np.all(np.isfinite(stack),axis=0)
    if int(common.sum())<1000:raise ValueError('Insufficient common pixels')
    spread=np.ptp(stack[:,common],axis=0)*1000
    result={'assumption':'Selected image ROI is one stationary flat surface; no external ruler ground truth',
        'normalized_roi_xyxy':list(roi),
        'absolute_accuracy_verified':False,'rgbd_registration_verified':False,'samples':rows,
        'common_pixels':int(common.sum()),'temporal_pixel_range_median_mm':float(np.median(spread)),
        'temporal_pixel_range_p95_mm':float(np.percentile(spread,95))}
    (folder/'plane_quality.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--roi',type=float,nargs=4,default=(.25,.25,.75,.60))
    args=p.parse_args();run(args.folder,args.roi)
