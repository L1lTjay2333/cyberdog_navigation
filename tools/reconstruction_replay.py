"""Replay saved depth keyframes into bounded submaps; never access the robot."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruction_core import checked_pose, decode_depth_frame
from reconstruction_registration import register_clouds, apply_transform,sampled_cloud
from reconstruction_submaps import DiskSubmaps


def replay(session, output, refine=False, max_voxels=200000,target_window=1):
    if type(target_window) is not int or target_window not in (1,3,5):
        raise ValueError('Target window must be 1, 3 or 5')
    session = Path(session)
    files = sorted(session.glob('frame_*.json'))
    archive = session/'frames.jsonl'
    if not files and not archive.is_file():
        raise ValueError('No saved raw keyframes')

    def records():
        if archive.is_file():
            if archive.stat().st_size > 256*1024*1024:
                raise ValueError('Raw frame archive exceeds size limit')
            with archive.open(encoding='utf-8') as stream:
                for index, line in enumerate(stream, 1):
                    yield 'archive_%04d'%index, json.loads(line)
        else:
            for file in files:
                yield file.name, json.loads(file.read_text(encoding='utf-8'))

    store = DiskSubmaps(output, max_voxels=max_voxels)
    previous = None
    coordinate_frame = None
    stamp = 0.
    reports = []
    prior = np.eye(4)
    references=[]
    exhausted = True
    for name, record in records():
        checked_pose(record)
        origin = (record['frame'], record['source_frame'])
        if coordinate_frame is not None and origin != coordinate_frame:
            raise ValueError('Saved camera or world coordinate frame changed')
        if record['stamp'] <= stamp:
            raise ValueError('Saved timestamps are not increasing')
        coordinate_frame, stamp = origin, record['stamp']
        cloud = decode_depth_frame(record)
        started = time.perf_counter()
        quality = {'accepted':True, 'reason':'odometry_baseline'}
        if refine and previous is not None:
            initialized=apply_transform(cloud,prior)
            target=np.concatenate(references[-target_window:])
            quality = register_clouds(initialized,target)
            if quality['accepted']:
                candidate=np.asarray(quality['transform']).dot(prior)
                total_angle=float(np.linalg.norm(Rotation.from_matrix(candidate[:3,:3]).as_rotvec()))
                center=cloud.mean(axis=0)
                total_shift=float(np.linalg.norm(apply_transform(center[None],candidate)[0]-center))
                quality.update(total_correction_at_centroid_m=total_shift,total_correction_angle_deg=float(np.degrees(total_angle)))
                if total_shift>.05 or total_angle>np.radians(5):
                    quality=dict(quality,accepted=False,reason='cumulative_correction_limit',
                                 transform=np.eye(4).tolist(),applied_correction_angle_deg=0.,
                                 applied_correction_at_centroid_m=0.)
        row = {'file':name, 'stamp':stamp, 'quality':quality,
               'registration_seconds':time.perf_counter()-started}
        if refine and not quality['accepted']:
            row['fused'] = False
            reports.append(row)
            exhausted = False
            # Stop instead of matching across an uncontrolled gap or silently
            # falling back to raw odometry after geometric tracking is lost.
            break
        if refine and previous is not None:
            prior=candidate
            cloud = apply_transform(cloud,prior)
            row['cumulative_correction']=prior.tolist()
        store.add(cloud)
        previous = cloud
        if refine:
            references.append(sampled_cloud(cloud))
            references=references[-target_window:]
        row['fused'] = True
        reports.append(row)
    manifest = store.finish()
    if coordinate_frame is None:
        raise ValueError('No raw frames in archive')
    metadata = json.loads((session/'summary.json').read_text(encoding='utf-8')) if (session/'summary.json').is_file() else {}
    archive_info = metadata.get('raw_archive') or {}
    full_replay = (archive.is_file() and exhausted and archive_info.get('complete') is True
                   and len(reports) == metadata.get('accepted_frames') == archive_info.get('frames'))
    report = {'input_keyframes':len(files) if not archive.is_file() else None,
              'input_source':'frames.jsonl' if archive.is_file() else 'saved_keyframes',
              'refinement_enabled':refine, 'frame':coordinate_frame[0],
              'registration_target_window':target_window,'cumulative_translation_limit_m':.05,
              'cumulative_angle_limit_deg':5.,
              'attempts':reports, 'submaps':manifest, 'control_requests':0,
              'online_enabled':False, 'absolute_accuracy_validated':False,
              'full_session_replay':bool(full_replay)}
    (Path(output)/'replay_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('output', type=Path, help='New output directory; never overwrite original captures')
    parser.add_argument('--refine', action='store_true')
    parser.add_argument('--max-voxels', type=int, default=200000)
    parser.add_argument('--target-window',type=int,choices=(1,3,5),default=1)
    args = parser.parse_args()
    result = replay(args.session,args.output,args.refine,args.max_voxels,args.target_window)
    print(json.dumps(result, indent=2))
