"""Audit excluded raw observations without declaring them ground-truth noise."""
import argparse
from collections import Counter
import hashlib
from itertools import zip_longest
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from reconstruction_core import decode_depth_frame
from reconstruction_postprocess import VoxelConsensus,read_depth,select_consensus,write_cloud


def classify_candidates(points,support,keep):
    labels=np.full(len(points),2,dtype=np.int8)
    confirmed=support>=2
    distances=np.full(len(points),np.inf)
    if confirmed.any():distances=cKDTree(points[confirmed]).query(points)[0]
    labels[(~confirmed)&(distances<=.03)]=1
    labels[confirmed&~keep]=3
    labels[keep]=0
    return labels,distances


def edge_flags(depth):
    valid=np.isfinite(depth)&(depth>=.15)&(depth<=4.)
    pad=np.pad(depth,1,constant_values=np.nan);edges=np.zeros(depth.shape,dtype=bool)
    for y,x in ((0,1),(2,1),(1,0),(1,2)):
        neighbor=pad[y:y+depth.shape[0],x:x+depth.shape[1]]
        with np.errstate(invalid='ignore'):
            edges|=valid&np.isfinite(neighbor)&(neighbor>=.15)&(neighbor<=4.)&(abs(depth-neighbor)>.04)
    return edges


def audit(session,processed,output):
    session,processed,output=Path(session),Path(processed),Path(output)
    report=json.loads((processed/'processing_report.json').read_text(encoding='utf-8'))
    if hashlib.sha256((session/'frames.jsonl').read_bytes()).hexdigest()!=report['source_sha256']:
        raise ValueError('Original archive differs from processed source')
    if (processed/'frames_spatial.jsonl').stat().st_size>256*1024*1024:raise ValueError('Archive too large')
    model=VoxelConsensus()
    with (processed/'frames_spatial.jsonl').open(encoding='utf-8') as stream:
        for i,line in enumerate(stream):
            if i>=900:raise ValueError('Too many frames')
            model.add(decode_depth_frame(json.loads(line)))
    points,keys,support=model.arrays();keep=select_consensus(points,support)
    if len(points)!=report['candidate_voxels'] or int(keep.sum())!=report['processed_points']:
        raise ValueError('Derived data does not reproduce processing report')
    labels,distances=classify_candidates(points,support,keep)
    key_labels={tuple(key):int(label) for key,label in zip(keys,labels)}
    names=['retained','single_near_confirmed','single_far_from_confirmed','confirmed_sparse']
    output.mkdir(parents=True,exist_ok=False)
    for code,name in enumerate(names):write_cloud(output/(name+'.ply'),points[labels==code],support[labels==code])
    frame_rows=[];selected={0,10,report['frames']//2,report['frames']-1};images=[]
    bands={name:Counter() for name in ('0.15-0.5m','0.5-1m','1-2m','2-4m')}
    # 0 retained, 1/2 uncertain single-frame, 3 confirmed but sparse, 4 image spike.
    palette=np.array([[0,0,0],[70,200,130],[245,210,60],[245,145,55],[200,90,220],[240,65,65]],dtype=np.uint8)
    with (session/'frames.jsonl').open(encoding='utf-8') as original,(processed/'frames_spatial.jsonl').open(encoding='utf-8') as spatial:
        for index,pair in enumerate(zip_longest(original,spatial)):
            if None in pair:raise ValueError('Frame archives have different lengths')
            raw,filtered=map(json.loads,pair)
            if raw['stamp']!=filtered['stamp']:raise ValueError('Frame timestamps changed')
            _,depth=read_depth(raw);_,clean=read_depth(filtered)
            z=depth[::4,::4];cz=clean[::4,::4]
            valid=np.isfinite(z)&(z>=.15)&(z<=4.)
            spatial_valid=np.isfinite(cz)&(cz>=.15)&(cz<=4.)
            codes=np.full(z.shape,-1,dtype=np.int8);codes[valid&~spatial_valid]=4
            world=decode_depth_frame(filtered)
            codes[spatial_valid]=[key_labels[tuple(key)] for key in model.keys(world)]
            edge=edge_flags(z)
            row={'frame':index+1,'valid_observations':int(valid.sum()),
                 'kept':int((codes==0).sum()),'single_near':int((codes==1).sum()),
                 'single_far':int((codes==2).sum()),'confirmed_sparse':int((codes==3).sum()),
                 'image_spike':int((codes==4).sum()),'edge_observations':int((edge&valid).sum()),
                 'kept_edge_observations':int((edge&(codes==0)).sum())}
            row['retained_fraction']=row['kept']/max(1,row['valid_observations'])
            frame_rows.append(row)
            for name,lo,hi in [('0.15-0.5m',.15,.5),('0.5-1m',.5,1.),('1-2m',1.,2.),('2-4m',2.,4.00001)]:
                mask=valid&(z>=lo)&(z<hi)
                bands[name].update(total=int(mask.sum()),kept=int((mask&(codes==0)).sum()))
            if index in selected:
                import open3d as o3d
                gray=np.zeros(z.shape,dtype=np.uint8)
                gray[valid]=(30+200*np.clip((4-z[valid])/3.85,0,1)).astype(np.uint8)
                color=palette[codes+1]
                for suffix,image in [('depth',np.repeat(gray[:,:,None],3,axis=2)),('classes',color)]:
                    image=np.repeat(np.repeat(image,4,axis=0),4,axis=1)
                    name='frame_%03d_%s.png'%(index+1,suffix)
                    if not o3d.io.write_image(str(output/name),o3d.geometry.Image(image)):raise OSError('PNG export failed')
                images.append(index+1)
    counts={name:int((labels==i).sum()) for i,name in enumerate(names)}
    result={'candidate_classes':counts,'frames':frame_rows,'depth_bands':bands,
            'observation_retention':sum(r['kept'] for r in frame_rows)/max(1,sum(r['valid_observations'] for r in frame_rows)),
            'near_confirmed_distance_m':.03,'semantic_labels_available':False,'false_removal_rate_known':False,
            'limitations':['Proximity to confirmed geometry is a review hint, not proof a point is true.',
                          'Without RGB/ground truth this audit cannot identify chair legs or measure true false-removal rate.',
                          'Distant single observations may be legitimate newly seen surfaces.']}
    (output/'audit_report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    gallery=''.join('<h2>第%d帧</h2><div class="pair"><figure><img src="frame_%03d_depth.png"><figcaption>原始深度灰度，不是RGB</figcaption></figure><figure><img src="frame_%03d_classes.png"><figcaption>数据分类，非语义识别</figcaption></figure></div>'%(i,i,i) for i in images)
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>采集数据保留审查</title><style>body{background:#17202b;color:#eef;font:16px system-ui;margin:24px;line-height:1.7}.pair{display:flex;flex-wrap:wrap}figure{margin:8px;max-width:48%}img{width:100%}a{color:#9cf}</style><h1>建模前数据保留审查</h1><p>绿色：多帧确认保留；黄色：单帧、距确认表面3cm内；橙色：单帧、较远；紫色：多帧但空间邻域不足；红色：深度孤立值；黑色：无效值。</p><p>排除不等于噪声。这里没有RGB，不能自动认定某个点是桌腿或门框，也无法给出真实误删率。黄色应重点复核；橙色也可能是新扫到的真实区域。</p><pre>'''+json.dumps(counts,ensure_ascii=False,indent=2)+'''</pre><a href="audit_report.json">完整分类及逐帧报告</a>'''+gallery+'</html>'
    (output/'index.html').write_text(html,encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('session','processed','output'):parser.add_argument(name,type=Path)
    args=parser.parse_args();result=audit(args.session,args.processed,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='frames'},ensure_ascii=False,indent=2))
