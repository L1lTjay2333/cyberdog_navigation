"""Real-photo SfM diagnostic; no invented RGB/depth calibration or robot access."""
import base64,hashlib,json,zlib,time,sys
from pathlib import Path
from PIL import Image
import pycolmap

def run(session,output):
 session,output=Path(session),Path(output);output.mkdir(parents=True,exist_ok=False)
 images=output/'images';images.mkdir();rows=[]
 source=session/'frames.jsonl';sha=hashlib.sha256(source.read_bytes()).hexdigest()
 for index,line in enumerate(source.open(encoding='utf-8'),1):
  frame=json.loads(line);c=frame.get('color_sample')
  if not c:continue
  raw=zlib.decompress(base64.b64decode(c['data_zlib_b64']))
  image=Image.frombytes('RGB',(c['width'],c['height']),raw,'raw','BGR' if c['encoding']=='bgr8' else 'RGB',c['step'])
  name='frame_%06d.png'%index;image.save(images/name)
  rows.append(dict(file=name,index=index,stamp=c['stamp']))
 (output/'photos.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
 def status(phase,**values):(output/'progress.json').write_text(json.dumps(dict(phase=phase,**values),indent=2),encoding='utf-8')
 status('features',photos=len(rows))
 extraction=pycolmap.FeatureExtractionOptions();extraction.num_threads=4
 pycolmap.extract_features(str(output/'database.db'),str(images),camera_mode=pycolmap.CameraMode.SINGLE,device=pycolmap.Device.cpu,extraction_options=extraction)
 status('matching',photos=len(rows))
 pycolmap.match_exhaustive(str(output/'database.db'),device=pycolmap.Device.cpu)
 status('mapping',photos=len(rows));sparse=output/'sparse';sparse.mkdir()
 models=pycolmap.incremental_mapping(str(output/'database.db'),str(images),str(sparse))
 result=dict(source_session=session.name,source_sha256=sha,input_photos=len(rows),models=len(models),rgb_depth_calibration_verified=False,metric_scale_verified=False,depth_model_replaced=False)
 if models:
  best_id,best=max(models.items(),key=lambda m:m[1].num_reg_images())
  best.export_PLY(str(output/'photo_sparse.ply'))
  result.update(best_model=int(best_id),registered_photos=best.num_reg_images(),points3D=best.num_points3D(),mean_reprojection_error=best.compute_mean_reprojection_error())
  # Region-of-image selection is a diagnostic, not object segmentation.
  selected=[]
  for point in best.points3D.values():
   hits=0
   for element in point.track.elements:
    image=best.images[element.image_id];xy=image.points2D[element.point2D_idx].xy;camera=best.cameras[image.camera_id]
    if xy[1]>.55*camera.height:hits+=1
   if hits>=2 and point.error<=3:selected.append(point)
  with (output/'lower_image_candidates.ply').open('w') as f:
   f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%len(selected))
   for p in selected:f.write(' '.join(map(str,[*p.xyz,*p.color]))+'\n')
  result['lower_image_candidates']=len(selected)
 if hashlib.sha256(source.read_bytes()).hexdigest()!=sha:raise ValueError('Source changed')
 (output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');status('done',results=result);print(json.dumps(result))

if __name__=='__main__':run(*sys.argv[1:])
