"""Offline Open3D installation smoke check. No robot/network operations."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import open3d as o3d
import scipy


def check(model):
    cloud = o3d.io.read_point_cloud(str(model))
    if not len(cloud.points) or not np.isfinite(np.asarray(cloud.points)).all():
        raise ValueError('Model is empty or non-finite')
    sampled = cloud.voxel_down_sample(.02)
    sampled.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.08, max_nn=30))
    if not np.isfinite(np.asarray(sampled.normals)).all():
        raise ValueError('Non-finite estimated normals')
    return {'python':sys.executable, 'open3d':o3d.__version__, 'numpy':np.__version__,
            'scipy':scipy.__version__, 'model':str(model.resolve()), 'input_points':len(cloud.points),
            'downsampled_points':len(sampled.points), 'finite_normals':True, 'control_requests':0,
            'scope':'Installation smoke check, not accuracy or real-time performance validation'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = check(args.model)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    print(text)
