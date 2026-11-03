"""Offline regression replay. Reads frozen room archives, never contacts robot."""
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from local_cloud_registration import LocalRegistration, estimate
from reconstruction_core import decode_depth_frame

ROOT = Path(__file__).resolve().parents[1]


def run():
    source = ROOT / 'evidence/live_room_processed_20261103'
    output = ROOT / 'evidence/room_registration_replay'
    output.mkdir(exist_ok=True)
    registration = LocalRegistration()
    records = []
    cells = {}
    accepted = []
    with (output / 'corrections.jsonl').open('w', encoding='utf-8') as log:
        for path in sorted(source.glob('session*.jsonl')):
            with path.open(encoding='utf-8') as stream:
                for line in stream:
                    record = json.loads(line)
                    record['aligned'] = True
                    registration.apply(record, record['key'])
                    info = record['registration']
                    records.append((record['stamp'], info))
                    if info['applied']:
                        accepted.append(info)
                    log.write(json.dumps({'stamp': record['stamp'], **info}) + '\n')
                    batch = {}
                    for point in record['points_xyz']:
                        key = tuple(np.floor(np.asarray(point) / .05).astype(int))
                        batch.setdefault(key, np.asarray(point))
                    for key, point in batch.items():
                        total, count = cells.get(key, (np.zeros(3), 0))
                        cells[key] = (total + point, count + 1)
    points = np.asarray([total / count for total, count in cells.values()])
    o3d.io.write_point_cloud(str(output / 'replayed.ply'),
                            o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
    points.astype('<f4').tofile(output / 'replayed.bin')
    pairs = np.asarray([[row['before'], row['after']] for row in accepted])
    # Check whether newly archived full-depth data actually includes movement.
    path = max((ROOT / 'evidence/live_cloud_maps').glob('session_*/frames.jsonl'),
               key=lambda p: p.stat().st_mtime)
    limit = path.stat().st_size
    poses = []
    decoded = 0
    failed = 0
    with path.open('rb') as stream:
        while stream.tell() < limit:
            line = stream.readline(limit - stream.tell())
            if not line.endswith(b'\n'):
                break
            depth = json.loads(line).get('depth')
            if depth:
                try:
                    decode_depth_frame(depth, stride=8)
                    poses.append(depth['translation'])
                    decoded += 1
                except ValueError:
                    failed += 1
    spread = np.ptp(np.asarray(poses), axis=0).tolist() if poses else []
    report = {
        'frames': len(records), 'accepted_corrections': len(accepted),
        'unchanged_frames': len(records) - len(accepted),
        'withheld_residual_median_m': np.median(pairs, axis=0).tolist() if len(pairs) else None,
        'maximum_translation_m': max((np.hypot(r['dx'], r['dy']) for r in accepted), default=0),
        'maximum_yaw_degrees': max((abs(np.degrees(r['yaw'])) for r in accepted), default=0),
        'new_depth_source': str(path.relative_to(ROOT)), 'decoded_depth_frames': decoded,
        'invalid_depth_frames': failed, 'camera_position_extent_m': spread,
        'limitations': ['Per-frame local correction, no accumulated pose-graph correction.',
                        'Residual measures local alignment, not absolute range accuracy.',
                        'Old archive lacks camera poses, so straight/turn/loop labels are unknown.',
                        'Corridor parallelism and loop closure are not yet verified.']}
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    run()
