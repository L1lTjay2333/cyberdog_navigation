"""Offline comparison on identical source samples, normals and held-out points.

Open3D's fitness/RMSE alone are not a validation gate. No robot or online change.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from reconstruction_core import checked_pose, decode_depth_frame
from reconstruction_registration import sampled_cloud, surface_geometry, apply_transform, register_clouds


def assess_candidate(source, target, transform):
    """Common held-out geometry checks; acceptability is not absolute accuracy."""
    transform = np.asarray(transform, dtype=float)
    rejected = {'qualified':False}
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-8)
            or not np.allclose(transform[:3, :3].T.dot(transform[:3, :3]), np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-6)):
        return dict(rejected, reason='invalid_rigid_transform')
    if min(len(source), len(target)) < 100:
        return dict(rejected, reason='too_few_points')
    tree, normals, good = surface_geometry(target)
    def metrics(points):
        distance, index = tree.query(points)
        valid = (distance < .08) & good[index]
        residual = np.einsum('ij,ij->i', normals[index], points-target[index])
        return index, valid, residual
    index, valid, before_residual = metrics(source)
    holdout = np.arange(len(source)) % 5 == 0
    evaluation, fit = holdout & valid, ~holdout & valid
    if evaluation.sum() < 30 or fit.sum() < 100 or valid.mean() < .45:
        return dict(rejected, reason='insufficient_overlap')
    origin = source.mean(axis=0)
    n = normals[index[fit]]
    jacobian = np.column_stack((np.cross(source[fit]-origin, n), n))
    weights = np.minimum(1., .01/np.maximum(abs(before_residual[fit]), 1e-12))
    singular = np.linalg.svd(jacobian*np.sqrt(weights)[:, None], compute_uv=False)
    ratio = float(singular[-1]/singular[0])
    _, after_valid, after_residual = metrics(apply_transform(source, transform))
    before = float(np.sqrt(np.mean(before_residual[evaluation]**2)))
    after = float(np.sqrt(np.mean(after_residual[evaluation]**2)))
    angle = float(np.linalg.norm(Rotation.from_matrix(transform[:3, :3]).as_rotvec()))
    shift = float(np.linalg.norm(apply_transform(origin[None], transform)[0]-origin))
    result = dict(rejected, before_rmse_m=before, after_rmse_m=after,
                  before_overlap=float(valid.mean()), after_overlap=float(after_valid.mean()),
                  retained_validation_overlap=float(after_valid[evaluation].mean()),
                  observability_ratio=ratio, correction_deg=math.degrees(angle), correction_m=shift,
                  validation_points=int(evaluation.sum()))
    if ratio < .01:
        result['reason'] = 'degenerate_geometry'
    elif shift > .05 or angle > math.radians(5):
        result['reason'] = 'excessive_correction'
    elif after_valid.mean() < .45 or after_valid[evaluation].mean() < .95:
        result['reason'] = 'lost_validation_overlap'
    elif after > .015 or after > before+.0002:
        result['reason'] = 'validation_residual_too_large'
    elif before > .002 and after > before*.95:
        if before <= .012 and shift <= .003 and angle <= math.radians(.3):
            result.update(qualified=True, reason='consistent_without_significant_gain')
        else:
            result['reason'] = 'no_significant_improvement'
    else:
        result.update(qualified=True, reason='qualified_local_candidate')
    return result


def compare_pair(source, target):
    source, target = sampled_cloud(source), sampled_cloud(target)
    start = time.perf_counter()
    custom = register_clouds(source, target)
    custom_seconds = time.perf_counter()-start
    if min(len(source), len(target)) < 100:
        return {'custom':custom, 'open3d':{'skipped':'too_few_points'}}
    _, normals, good = surface_geometry(target)
    # Match the custom solver: do not use held-out source points during fitting.
    training = source[np.arange(len(source)) % 5 != 0]
    src, dst = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(training)
    dst.points = o3d.utility.Vector3dVector(target)
    # Keep the same neighbor geometry. Invalid normals have zero weight instead
    # of changing the set of target nearest neighbors between solvers.
    dst.normals = o3d.utility.Vector3dVector(np.where(good[:, None], normals, 0.))
    start = time.perf_counter()
    registration = o3d.pipelines.registration
    candidate = registration.registration_icp(
        src, dst, .08, np.eye(4),
        registration.TransformationEstimationPointToPlane(registration.HuberLoss(.01)),
        registration.ICPConvergenceCriteria(max_iteration=20))
    elapsed = time.perf_counter()-start
    return {'identity':assess_candidate(source, target, np.eye(4)),
            'custom':{'solver_result':custom, 'seconds':custom_seconds,
                      'common_check':assess_candidate(source, target, custom['transform'])},
            'open3d':{'transform':candidate.transformation.tolist(), 'seconds':elapsed,
                      'solver_fitness':candidate.fitness, 'solver_inlier_rmse_m':candidate.inlier_rmse,
                      'common_check':assess_candidate(source, target, candidate.transformation)}}


def benchmark(session):
    session = Path(session)
    archive = session/'frames.jsonl'
    if archive.is_file():
        if archive.stat().st_size > 64*1024*1024:
            raise ValueError('Archive exceeds supported size')
        frames = []
        with archive.open(encoding='utf-8') as stream:
            for line in stream:
                frames.append(json.loads(line))
                if len(frames) > 360:
                    raise ValueError('Archive exceeds supported frame count')
        names = ['archive_%04d'%(i+1) for i in range(len(frames))]
    else:
        files = sorted(session.glob('frame_*.json'))
        frames = [json.loads(file.read_text(encoding='utf-8')) for file in files]
        names = [file.name for file in files]
    if not 2 <= len(frames) <= 360:
        raise ValueError('Benchmark expects 2 to 360 frames')
    for frame in frames:checked_pose(frame)
    if len({(f['frame'], f['source_frame']) for f in frames}) != 1:
        raise ValueError('Camera or world coordinate frame changed')
    if any(b['stamp'] <= a['stamp'] for a,b in zip(frames, frames[1:])):
        raise ValueError('Keyframe timestamps are not increasing')
    clouds = [decode_depth_frame(frame) for frame in frames]
    pairs = list(zip(range(len(frames)-1), range(1, len(frames))))
    if len(frames)>2:pairs.append((0, len(frames)-1))
    rows = []
    for i,j in pairs:
        rows.append(dict(compare_pair(clouds[j], clouds[i]), reference=names[i],
                         source=names[j], gap_s=frames[j]['stamp']-frames[i]['stamp']))
    metadata = json.loads((session/'summary.json').read_text(encoding='utf-8')) if (session/'summary.json').is_file() else {}
    archive_info = metadata.get('raw_archive') or {}
    complete = (archive.is_file() and archive_info.get('complete') is True
                and len(frames) == archive_info.get('frames') == metadata.get('accepted_frames'))
    return {'open3d':o3d.__version__, 'session':str(Path(session).resolve()), 'pairs':rows,
            'input_frames':len(frames), 'input_source':'frames.jsonl' if archive.is_file() else 'keyframes',
            'all_accepted_frames_evaluated':bool(complete), 'capture_termination':metadata.get('error'),
            'online_enabled':False, 'control_requests':0, 'full_session_replay':False,
            'absolute_accuracy_validated':False,
            'limitations':['Independent frame-pair evaluations, not a cumulatively corrected trajectory.',
                          'Archive completeness covers accepted frames, not motion after capture termination.',
                          'Held-out source samples are not independent metrology.',
                          'Solvers share samples/normals/Huber scale, but differ in iteration and convergence rules.',
                          'Custom output is the guarded applied transform; rejected internal proposals are not reconstructed.',
                          'Timing excludes some preprocessing and does not establish real-time throughput.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = benchmark(args.session)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    for row in result['pairs']:
        print(row['reference'], row['source'], json.dumps(row['open3d'].get('common_check', row['open3d'])))
