"""Compare single-frame and fused surfaces on depth rays excluded from fusion."""
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from reconstruction_mesh import depth_and_camera
from reconstruction_tsdf import fuse_depth_records

def scene(mesh):
    result = o3d.t.geometry.RaycastingScene()
    result.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return result

def run():
    root = Path(__file__).resolve().parents[1]/'evidence/offline_depth_room_check_20261104'
    rows = [json.loads(line) for line in (root/'depth_frames.jsonl').read_text().splitlines()]
    train = rows[:40]
    single = scene(fuse_depth_records(train[:1])[0])
    fused = scene(fuse_depth_records(train)[1])
    errors = [[], []]
    valid_pixels = 0
    for row in rows[40:]:
        depth, K, extrinsic = depth_and_camera(row)
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(K.intrinsic_matrix,extrinsic,row['width'],row['height']).numpy()[::8,::8].copy()
        measured = depth[::8,::8]
        valid = (measured>=.15) & (measured<=4.)
        valid_pixels += int(valid.sum())
        cast = [s.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy() for s in (single,fused)]
        mask = valid & np.isfinite(cast[0]) & np.isfinite(cast[1])
        for out, prediction in zip(errors,cast):
            out.extend(abs(prediction[mask]-measured[mask]))
    report = dict(train_frames=len(train),held_out_frames=len(rows)-len(train),shared_pixels=len(errors[0]),
                  valid_pixels=valid_pixels,errors_mm={name:np.percentile(values,[50,90,95]).tolist()
                  for name,values in zip(['single','fused'],[np.array(x)*1000 for x in errors])},
                  quantiles=[50,90,95])
    (root/'held_out_depth.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
if __name__=='__main__':
    run()
