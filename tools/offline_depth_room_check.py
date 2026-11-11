"""Local depth fusion and endpoint overlap audit using saved files only."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d
from reconstruction_tsdf import fuse_depth_records
from reconstruction_core import decode_depth_frame

ROOT = Path(__file__).resolve().parents[1]


def endpoint_overlap():
    records = []
    for path in sorted((ROOT / 'evidence/live_room_processed_20261103').glob('session*.jsonl')):
        with path.open(encoding='utf-8') as stream:
            records.extend(json.loads(line) for line in stream)
    def merged(rows):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
            np.concatenate([np.asarray(row['points_xyz']) for row in rows])))
        return np.asarray(cloud.voxel_down_sample(.05).points)
    first, last = merged(records[:5]), merged(records[-5:])
    distances = [cKDTree(last).query(first)[0], cKDTree(first).query(last)[0]]
    return dict(first_to_last_overlap_10cm=float(np.mean(distances[0] < .1)),
                last_to_first_overlap_10cm=float(np.mean(distances[1] < .1)),
                loop_correction_applied=False,
                explanation='Endpoint surface overlap alone cannot confirm a revisited robot pose.')


def run():
    output = ROOT / 'evidence/offline_depth_room_check_20261104'
    output.mkdir(exist_ok=True)
    source = max((ROOT / 'evidence/live_cloud_maps').glob('session_*/frames.jsonl'),
                 key=lambda path: path.stat().st_mtime)
    records = []
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            record = json.loads(line).get('depth')
            if record:
                records.append(record)
                if len(records) == 60:
                    break
    if len(records) < 3:
        raise ValueError('Need at least three saved depth frames')
    with (output / 'depth_frames.jsonl').open('w', encoding='utf-8') as stream:
        for record in records:
            stream.write(json.dumps(record) + '\n')
    observed, supported, fusion = fuse_depth_records(records)
    raw = np.concatenate([decode_depth_frame(record, stride=8) for record in records])
    raw_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(raw)).voxel_down_sample(.02)
    np.asarray(raw_cloud.points).astype('<f4').tofile(output / 'raw.bin')
    for name, mesh in [('observed', observed), ('supported', supported)]:
        o3d.io.write_triangle_mesh(str(output / (name + '.ply')), mesh)
        vertices = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.triangles)
        vertices[faces].astype('<f4').tofile(output / (name + '.bin'))
    positions = np.asarray([record['translation'] for record in records])
    report = dict(source=str(source.relative_to(ROOT)), frames=len(records),
                  position_extent_m=np.ptp(positions, axis=0).tolist(),
                  fusion=fusion, endpoint_audit=endpoint_overlap(),
                  scope='Static local depth fusion trial, separate from full room mesh.')
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='fusion'}), flush=True)
    print(json.dumps(fusion['higher_support']), flush=True)


if __name__ == '__main__':
    run()
