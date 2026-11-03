"""Crop a completed offline result to its recorded target box; retain full-scene exports."""
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from reconstruction_offline_worker import preview_mesh,progress

def crop_result(session,output):
    session,output=Path(session),Path(output)
    summary=json.loads((session/'summary.json').read_text(encoding='utf-8'))
    target=summary.get('target_box')
    if not target or target.get('center') is None:return
    center,size=np.array(target['center']),np.array(target['size'])
    if center.shape!=(3,) or size.shape!=(3,) or not np.isfinite([center,size]).all() or np.any(size<=0):raise ValueError('Invalid target box')
    bounds=o3d.geometry.AxisAlignedBoundingBox(center-size/2,center+size/2)
    result=json.loads((output/'result.json').read_text(encoding='utf-8'))
    previews={}
    for name in ('observed','supported'):
        path=output/'fusion'/(name+'_gray.ply')
        full=path.with_name(name+'_full_scene.ply')
        if not path.exists():continue
        if not full.exists():path.rename(full)
        mesh=o3d.io.read_triangle_mesh(str(full)).crop(bounds)
        if not len(mesh.triangles):continue
        if not o3d.io.write_triangle_mesh(str(path),mesh):raise OSError('Target mesh write failed')
        previews[name]=preview_mesh(path,output,name)
        result[name+'_triangles']=len(mesh.triangles)
    with np.load(output/'processed/points_processed.npz',allow_pickle=False) as saved:
        points=saved['points']
    points=points[np.all(np.abs(points-center)<=size/2,axis=1)]
    if len(points):
        preview=points[np.linspace(0,len(points)-1,min(30000,len(points))).astype(int)].astype('<f4')
        preview.tofile(output/'points.bin')
        path=output/'processed/points_processed.ply';full=path.with_name('points_full_scene.ply')
        if not full.exists():path.rename(full)
        o3d.io.write_point_cloud(str(path),o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
        previews['points']={'file':'points.bin','kind':'points','displayed_points':len(preview),'full_points':len(points),'position_count':len(preview),'minimum':points.min(axis=0).tolist(),'maximum':points.max(axis=0).tolist()}
    result.update(full_scene_processed_points=result['processed_points'],processed_points=len(points),previews=previews,target_box=target,note='按本轮保存的中心范围裁剪预览；全场景结果另存。使用原始里程计位姿，未完成全轨迹纠偏或绝对精度验收。')
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    progress(output,'done','全部帧处理完成，显示中心目标范围',results=result)
    return result

if __name__=='__main__':
    import sys
    print(json.dumps(crop_result(*sys.argv[1:]),ensure_ascii=False))
