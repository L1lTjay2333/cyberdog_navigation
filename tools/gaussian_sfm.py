"""CPU COLMAP preprocessing in the separate D-drive environment; no robot."""
import argparse,json
from pathlib import Path
import pycolmap


def run(folder):
    root=Path(folder);report=json.loads((root/'dataset.json').read_text(encoding='utf-8'))
    if not report['multiview_screen_pass']:
        raise ValueError('多视角数据不足：'+'；'.join(report['screen_reasons']))
    database=root/'database.db';sparse=root/'sparse'
    if database.exists() or sparse.exists():raise ValueError('Use a fresh exported dataset')
    sparse.mkdir()
    pycolmap.extract_features(database_path=str(database),image_path=str(root/'images'),
                             camera_mode=pycolmap.CameraMode.SINGLE,device=pycolmap.Device.cpu)
    pycolmap.match_exhaustive(database_path=str(database),device=pycolmap.Device.cpu)
    models=pycolmap.incremental_mapping(database_path=str(database),image_path=str(root/'images'),output_path=str(sparse))
    if not models:raise ValueError('COLMAP未建立相机模型；需检查照片清晰度、纹理和重叠')
    best_id,best=max(models.items(),key=lambda item:item[1].num_reg_images())
    registered=best.num_reg_images();fraction=registered/report['photos']
    result=dict(model=str((sparse/str(best_id)).resolve()),registered_images=registered,
        input_images=report['photos'],registered_fraction=fraction,points3D=best.num_points3D(),
        reconstruction_screen_pass=registered>=24 and fraction>=.8 and best.num_points3D()>=100,
        geometry_accuracy_verified=False,metric_scale_verified=False)
    (root/'sfm_report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if not result['reconstruction_screen_pass']:raise ValueError('照片位姿重建覆盖不足，未放行训练')
    pycolmap.undistort_images(output_path=str(root/'undistorted'),input_path=str(sparse/str(best_id)),
                              image_path=str(root/'images'))
    result['undistorted_dataset']=str((root/'undistorted').resolve())
    (root/'sfm_report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');args=p.parse_args();print(json.dumps(run(args.dataset)))
