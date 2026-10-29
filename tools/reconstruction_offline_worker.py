"""Owned offline processing worker. No robot, SSH, ROS or network interfaces."""
import argparse
import json
from pathlib import Path
import time
import traceback
import numpy as np
import open3d as o3d
from reconstruction_postprocess import process_capture
from reconstruction_tsdf import build


def progress(output,phase,message,**extra):
    data=dict(phase=phase,message=message,updated_unix=time.time(),**extra)
    temporary=output/'progress.tmp'
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    temporary.replace(output/'progress.json')


def preview_mesh(path,output,name):
    if not path.is_file():return None
    mesh=o3d.io.read_triangle_mesh(str(path))
    faces=np.asarray(mesh.triangles);vertices=np.asarray(mesh.vertices)
    if not len(faces):return None
    # Dropping individual faces creates artificial holes. Simplify only the
    # display mesh; the full exported PLY and source geometry remain untouched.
    preview=mesh.simplify_quadric_decimation(150000) if len(faces)>150000 else mesh
    selected=np.asarray(preview.triangles)
    if len(selected)>150000:raise ValueError('Preview simplification exceeded face budget')
    positions=np.asarray(preview.vertices)[selected].astype('<f4').reshape(-1,3)
    if not np.isfinite(positions).all():raise ValueError('Non-finite preview geometry')
    positions.tofile(output/(name+'.bin'))
    return {'file':name+'.bin','kind':'mesh','preview_method':'quadric_decimation' if len(faces)>150000 else 'full_mesh',
            'displayed_triangles':len(selected),'full_triangles':len(faces),
            'position_count':len(positions),'minimum':vertices.min(axis=0).tolist(),'maximum':vertices.max(axis=0).tolist()}


def run(session,output):
    progress(output,'processing','正在过滤深度数据、合并重复观测并保存待确认点')
    processed=process_capture(session,output/'processed')
    progress(output,'fusion','正在融合深度数据并提取灰色表面')
    fused=build(session,output/'fusion')
    if processed['source_sha256']!=fused['source_sha256']:raise ValueError('Source archive changed between stages')
    progress(output,'preview','正在准备预览和导出文件')
    previews={}
    for key in ('observed','supported'):
        value=preview_mesh(output/'fusion'/(key+'_gray.ply'),output,key)
        if value:previews[key]=value
    with np.load(output/'processed/points_processed.npz',allow_pickle=False) as saved:
        points=saved['points']
        sampled=points[np.linspace(0,len(points)-1,min(30000,len(points))).astype(int)].astype('<f4')
        sampled.tofile(output/'points.bin')
        if len(points):
            previews['points']={'file':'points.bin','kind':'points','displayed_points':len(sampled),
                                'full_points':len(points),'position_count':len(sampled),
                                'minimum':points.min(axis=0).tolist(),'maximum':points.max(axis=0).tolist()}
    result={'session':session.name,'source_sha256':fused['source_sha256'],'frames':fused['frames'],
            'processed_points':processed['processed_points'],'excluded_points':processed['excluded_points'],
            'observed_triangles':fused['all_observations']['triangles'],
            'supported_triangles':fused['higher_support']['triangles'],'previews':previews,
            'source_capture_termination':fused['source_capture_termination'],
            'absolute_accuracy_validated':False,'pose_correction':False,'mesh_smoothing':False,
            'note':'点云按共识分层保存；表面融合使用原深度及同样空间过滤，不硬删除低支持新视角。'}
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    progress(output,'done','离线处理完成；原始数据保留，精度仍待验收',results=result)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    try:run(args.session,args.output)
    except Exception as error:
        progress(args.output,'failed',str(error)[:1000]);traceback.print_exc();raise SystemExit(1)
