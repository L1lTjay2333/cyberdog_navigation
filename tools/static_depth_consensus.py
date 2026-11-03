"""Offline majority consensus for explicitly stationary, same-camera samples.

No spatial smoothing, inferred surfaces, pose correction or stock changes.
Rejects pixels unless a strict majority agrees within a bounded depth interval.
"""
import argparse
import base64
import json
from pathlib import Path
import warnings
import numpy as np
from capture_rgbd_samples import save_rgb_png


def consensus(stack, span=.02):
    a=np.asarray(stack,dtype=float)
    if a.ndim!=3 or not 3<=len(a)<=32:raise ValueError('Need 3..32 static depth frames')
    if not 0<span<=.1:raise ValueError('Invalid consensus depth span')
    valid=np.isfinite(a)&(a>=.15)&(a<=4)
    ordered=np.sort(np.where(valid,a,np.inf),axis=0)
    required=len(a)//2+1
    best=np.zeros(a.shape[1:],dtype=np.uint8)
    result=np.full(a.shape[1:],np.nan)
    for i in range(len(a)):
        lo=ordered[i]
        with np.errstate(invalid='ignore'):
            mask=np.isfinite(ordered)&np.isfinite(lo)&(ordered>=lo)&(ordered-lo<=span)
        count=mask.sum(axis=0)
        choose=(count>best)&(count>=required)
        # Median within the winning interval cannot bridge a 2-vs-2 split.
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',RuntimeWarning)
            middle=np.nanmedian(np.where(mask,ordered,np.nan),axis=0)
        result[choose]=middle[choose];best[choose]=count[choose]
    return result,best,required


def run(folder):
    metadata=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    samples=[s for s in metadata['samples'] if 'depth_file'in s]
    if len(samples)<3:raise ValueError('Insufficient archived frames; need at least 3')
    p=None;signature=None;frames=[];stamps=[]
    for s in samples:
        infos=[c for c in s['calibration'] if c['frame_id']==s['frame_id'] and
               c['width']==s['width'] and c['height']==s['height'] and abs(c['stamp']-s['stamp'])<=.2]
        if len(infos)!=1:raise ValueError('Need unique same-frame recent calibration')
        info=infos[0];projection=np.asarray(info['p'],dtype=float)
        sig=(s['topic'],s['frame_id'],s['width'],s['height'])
        if signature is not None and (sig!=signature or not np.allclose(projection,p,rtol=0,atol=1e-6)):
            raise ValueError('Camera geometry changed; cannot combine pixels')
        signature=sig;p=projection
        z=np.load(folder/s['depth_file']).astype(float)*(.001 if s['encoding']=='16UC1' else 1)
        if z.shape!=(s['height'],s['width']):raise ValueError('Invalid dimensions')
        frames.append(z);stamps.append(s['stamp'])
    if len(set(stamps))!=len(stamps) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Timestamps must be unique and increasing')
    stack=np.stack(frames);filtered,support,required=consensus(stack)
    output=folder/'static_consensus';output.mkdir(exist_ok=True)
    np.save(output/'depth_consensus_m.npy',filtered.astype(np.float32))
    np.save(output/'support.npy',support)
    valid=np.isfinite(stack)&(stack>=.15)&(stack<=4)
    common=np.all(valid,axis=0)
    ranges=np.ptp(stack[:,common],axis=0)
    accepted=np.isfinite(filtered)
    if len(p)!=12 or not np.isfinite(p).all() or p[0]<=0 or p[5]<=0 or p[3]!=0 or p[7]!=0:
        raise ValueError('Unsupported projection')
    v,u=np.nonzero(accepted);z=filtered[accepted]
    xyz=np.stack(((u-p[2])*z/p[0],(v-p[6])*z/p[5],z),axis=1)
    header='ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header'%len(xyz)
    np.savetxt(output/'consensus_camera_points.ply',xyz,fmt='%.6f',header=header,comments='')
    report={'assumption':'Robot, scene and camera extrinsics stationary throughout this session; operator context, not automatically verified',
        'frames':len(stack),'required_support':required,'max_cluster_span_m':.02,
        'camera_frame':signature[1],'accepted_pixels':int(accepted.sum()),
        'accepted_fraction':float(accepted.mean()),
        'raw_common_valid_pixels':int(common.sum()),
        'raw_pixel_range_median_mm':float(np.median(ranges)*1000) if len(ranges) else None,
        'raw_pixel_range_p95_mm':float(np.percentile(ranges,95)*1000) if len(ranges) else None,
        'raw_common_fraction_range_over_20mm':float(np.mean(ranges>.02)) if len(ranges) else None,
        'geometry_accuracy_verified':False,'world_pose_verified':False,
        'note':'Single stationary consensus surface; not an animated measurement, motion-compensated map or calibration fix'}
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    images=[]
    for i,z in enumerate(frames+[filtered]):
        good=np.isfinite(z)&(z>=.15)&(z<=4)
        x=np.clip((np.nan_to_num(z)-.15)/3.85,0,1)
        rgb=np.stack((255*x,255*(1-np.abs(2*x-1)),255*(1-x)),axis=-1).astype(np.uint8)
        rgb[~good]=0
        path=output/('raw_%s.png'%i if i<len(frames) else 'consensus.png')
        save_rgb_png(path,rgb);images.append(path)
    # Static HTML requires no JS and embeds the images for reliable app preview.
    def image(path):return '<img width="640" style="max-width:100%" src="data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode()+'">'
    html='<meta charset="utf-8"><title>同一静止批次：原始深度与多数一致结果</title><body style="background:#101923;color:white;font:16px/1.6 system-ui"><h1>同一静止批次：原始深度与多数一致结果</h1><p>记录 '+folder.name+'。相同颜色刻度：0.15–4米；黑色为无有效值。不是彩色照片对齐图。</p><h2>多数一致结果</h2><p>共%d帧，至少%d帧的深度落在同一20毫米区间才保留；无多数支持处留空。只适用于相机与场景静止。</p>'%(len(frames),required)+image(images[-1])
    for i,path in enumerate(images[:-1]):html+='<h2>原始深度 第%d帧</h2>'%(i+1)+image(path)
    html+='<p>平均或中位数不能修复错误标定、世界位姿跳变或所有帧共有的系统偏差；该结果不是绝对精度验收。</p>'
    (output/'index.html').write_text(html,encoding='utf-8')
    print(json.dumps({'folder':str(output),**report}))
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--static-confirmed',action='store_true',help='Operator confirms robot AND target remained stationary')
    args=p.parse_args()
    if not args.static_confirmed:p.error('Pixelwise consensus requires --static-confirmed')
    run(args.folder)
