"""PC-only experimental geometric registration. No ROS or network access.

Clouds are already in the odometry frame: fit only a bounded residual SE(3).
Single planes and low-overlap views are rejected rather than trusted as 6DoF.
"""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def cloud_array(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Expected finite Nx3 points')
    if len(points) > 100000:
        raise ValueError('Registration input exceeds point limit')
    return points


def sampled_cloud(points, voxel=.02, limit=6000):
    points = cloud_array(points)
    if not len(points):
        return points.copy()
    scaled = np.floor(points / voxel)
    if np.any(abs(scaled) > np.iinfo(np.int64).max // 2):
        raise ValueError('Cloud coordinates exceed bounds')
    _, indices = np.unique(scaled.astype(np.int64), axis=0, return_index=True)
    indices.sort()
    indices = indices[np.linspace(0, len(indices)-1, min(limit, len(indices))).astype(int)]
    return points[indices]


def apply_transform(points, transform):
    return np.asarray(points).dot(transform[:3, :3].T) + transform[:3, 3]


def register_clouds(source, target):
    """Robust point-to-plane ICP with observability and held-out quality gates.

    Returning accepted=False never authorizes fusion or a pose update. The
    uncorrected transform is returned on rejection; reasons remain inspectable.
    This is local refinement, not global relocalization or loop closure.
    """
    source, target = sampled_cloud(source), sampled_cloud(target)
    identity = np.eye(4)
    result = {'accepted':False, 'reason':'too_few_points', 'transform':identity.tolist(),
              'source_points':len(source), 'target_points':len(target),
              'applied_correction_angle_deg':0., 'applied_correction_at_centroid_m':0.}
    if min(len(source), len(target)) < 100:
        return result
    tree = cKDTree(target)
    distances, neighbors = tree.query(target, k=16)
    patches = target[neighbors]
    centered = patches - patches.mean(axis=1)[:, None, :]
    covariance = np.einsum('nki,nkj->nij', centered, centered) / 16
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    good_normals = (distances[:, -1] < .15) & (values[:, 1] > 1e-7) & (values[:, 0] < .12*values[:, 1])
    if good_normals.sum() < 100:
        result['reason'] = 'insufficient_local_surfaces'
        return result
    origin = source.mean(axis=0)
    held_out = np.arange(len(source)) % 5 == 0

    def matches(cloud):
        d, idx = tree.query(cloud)
        valid = (d < .08) & good_normals[idx]
        residual = np.einsum('ij,ij->i', normals[idx], cloud-target[idx])
        return d, idx, valid, residual

    _, _, initial_valid, initial_residual = matches(source)
    result['initial_overlap'] = float(initial_valid.mean())
    if initial_valid.mean() < .45:
        result['reason'] = 'insufficient_overlap'
        return result
    # Evaluate the same held-out points before and after, so losing bad matches
    # cannot manufacture apparent improvement.
    evaluation = initial_valid & held_out
    if evaluation.sum() < 30:
        result['reason'] = 'insufficient_validation_points'
        return result
    before = float(np.sqrt(np.mean(initial_residual[evaluation]**2)))
    result['before_validation_rmse_m'] = before
    current, transform = source.copy(), identity.copy()
    for iteration in range(20):
        _, idx, valid, residual = matches(current)
        fit = valid & ~held_out
        if fit.sum() < 100:
            result['reason'] = 'lost_overlap'
            return result
        p, n, r = current[fit]-origin, normals[idx[fit]], residual[fit]
        jacobian = np.column_stack((np.cross(p, n), n))
        weights = np.minimum(1., .01 / np.maximum(abs(r), 1e-12))
        weighted = jacobian * np.sqrt(weights)[:, None]
        singular = np.linalg.svd(weighted, compute_uv=False)
        ratio = float(singular[-1] / singular[0])
        result['observability_ratio'] = ratio
        if ratio < .01:
            result['reason'] = 'degenerate_geometry'
            return result
        increment = np.linalg.lstsq(weighted, -r*np.sqrt(weights), rcond=None)[0]
        if np.linalg.norm(increment[:3]) > math.radians(3) or np.linalg.norm(increment[3:]) > .03:
            result['reason'] = 'excessive_iteration_correction'
            return result
        rotation = Rotation.from_rotvec(increment[:3]).as_matrix()
        delta = np.eye(4)
        delta[:3, :3] = rotation
        delta[:3, 3] = origin - rotation.dot(origin) + increment[3:]
        transform = delta.dot(transform)
        current = apply_transform(source, transform)
        angle = float(np.linalg.norm(Rotation.from_matrix(transform[:3, :3]).as_rotvec()))
        center_shift = float(np.linalg.norm(apply_transform(origin[None], transform)[0]-origin))
        if angle > math.radians(5) or center_shift > .05:
            result['reason'] = 'excessive_total_correction'
            return result
        if np.linalg.norm(increment) < 1e-6:
            break
    _, _, final_valid, final_residual = matches(current)
    result.update(iterations=iteration+1, final_overlap=float(final_valid.mean()),
                  proposed_correction_angle_deg=math.degrees(angle), proposed_correction_at_centroid_m=center_shift,
                  validation_overlap_retained=float(final_valid[evaluation].mean()))
    if final_valid.mean() < .45 or final_valid[evaluation].mean() < .95:
        result['reason'] = 'lost_validation_overlap'
        return result
    after = float(np.sqrt(np.mean(final_residual[evaluation]**2)))
    result['candidate_validation_rmse_m'] = after
    if after > .015 or after > before + .0002:
        result['reason'] = 'validation_residual_too_large'
        return result
    if before > .002 and after > .95*before:
        # A tiny proposed correction with already consistent geometry is not
        # tracking loss. Certify the identity, instead of introducing a change
        # that failed the improvement test. This is distinct from blind fallback.
        if before <= .012 and center_shift <= .003 and angle <= math.radians(.3):
            result.update(accepted=True, reason='consistent_without_correction')
        else:
            result['reason'] = 'no_significant_improvement'
        return result
    result.update(accepted=True, reason='accepted_local_refinement', transform=transform.tolist(),
                  applied_correction_angle_deg=math.degrees(angle), applied_correction_at_centroid_m=center_shift)
    return result
