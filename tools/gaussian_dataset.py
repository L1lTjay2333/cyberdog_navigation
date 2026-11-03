"""Export genuine camera photos for COLMAP/gsplat; never invent RGB poses."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from reconstruction_color import photo
from reconstruction_core import checked_pose
from capture_rgbd_samples import save_rgb_png


def export(session,output):
    session,output=Path(session),Path(output)
    archive=session/'frames.jsonl'
    if not 0<archive.stat().st_size<=256*1024*1024:raise ValueError('Archive bound exceeded')
    source_hash=hashlib.sha256(archive.read_bytes()).hexdigest()
    output.mkdir(parents=True,exist_ok=False);images=output/'images';images.mkdir()
    samples=[];positions=[];seen=set();last=0.;camera=None
    with archive.open(encoding='utf-8') as source:
        lines=source.readlines()
    for index,line in enumerate(lines,1):
        if index>900:raise ValueError('Frame limit exceeded')
        f=json.loads(line);position,_=checked_pose(f)
        if f['stamp']<=last:raise ValueError('Unordered frame timestamps')
        last=f['stamp'];s=f.get('color_sample')
        if not s:continue
        if abs(s['stamp']-f['stamp'])>.05 or abs(s['depth_stamp']-f['stamp'])>1e-6:
            raise ValueError('Invalid temporal pair')
        rgb=photo(s);signature=rgb.shape
        if camera is not None and signature!=camera:raise ValueError('Color dimensions changed')
        camera=signature;digest=hashlib.sha256(rgb.tobytes()).hexdigest()
        if digest in seen:continue
        seen.add(digest);name='frame_%06d.png'%index;save_rgb_png(images/name,rgb)
        # Depth pose retained only as provenance. COLMAP must estimate RGB poses.
        samples.append(dict(file=name,source_frame_index=index,color_stamp=s['stamp'],
            depth_stamp=f['stamp'],pair_difference_s=abs(s['stamp']-f['stamp']),
            depth_position=position.tolist(),photo_sha256=digest))
        positions.append(position)
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=source_hash:raise ValueError('Source changed')
    displacement=float(np.linalg.norm(np.asarray(positions)-positions[0],axis=1).max()) if positions else 0.
    reasons=[]
    if len(samples)<30:reasons.append('至少准备30张清晰、有重叠的多角度照片；这是项目初筛门槛')
    if displacement<.15:reasons.append('深度相机位姿未显示足够平移；静止/原地旋转不能替代绕物采集')
    report=dict(source=str(archive.resolve()),source_sha256=source_hash,photos=len(samples),
        samples=samples,estimated_camera_displacement_m=displacement,
        multiview_screen_pass=not reasons,screen_reasons=reasons,
        rgb_poses_estimated=False,depth_registration_verified=False,training_ready=False,
        note='A preliminary screen only; COLMAP registration, overlap and held-out views must pass before training.')
    (output/'dataset.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session');p.add_argument('output')
    a=p.parse_args();r=export(a.session,a.output)
    print(json.dumps({k:v for k,v in r.items() if k!='samples'},ensure_ascii=True))
