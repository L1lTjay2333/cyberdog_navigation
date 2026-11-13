"""Random access to archived frames associated with saved portfolio entries."""
from functools import lru_cache
import json
from pathlib import Path
import re

import numpy as np
from reconstruction_core import project_camera_depth


BICYCLE_WORK = 'depth-20261110_170515_605008'
CAPTURE_ID = re.compile(r'\d{8}_\d{6}_\d{6}')
ROOM_ARCHIVES = {
    'map1640': 'session_1793867659_b7675ab9',
    'map1711': 'session_1793869892_f8d53c78',
}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def saved_work(root, work_id):
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', work_id):
        raise ValueError('作品编号无效')
    works = read_json(root/'assets/portfolio/manifest.json')
    for work in works:
        if work['id'] == work_id:
            return work
    raise ValueError('作品未保存或已移除')


def map_archive(root, name):
    candidates = []
    for archive in (root/'evidence/live_cloud_maps').glob('session_*/frames.jsonl'):
        with archive.open('rb') as stream:
            line = stream.readline()
        if line and json.loads(line).get('key','').split(':')[0] == name:
            candidates.append(archive)
    return sorted(candidates)


def archive_files(root, work):
    source = (root/work.get('source','')).resolve()
    if not source.is_relative_to((root/'evidence').resolve()):
        return []
    if work['id'] in ROOM_ARCHIVES:
        return [root/'evidence/live_cloud_maps'/ROOM_ARCHIVES[work['id']]/'frames.jsonl']
    if work['kind'] == 'map':
        return map_archive(root,source.name)
    capture = work.get('capture_id','')
    parts = source.relative_to(root/'evidence').parts
    if len(parts) >= 2 and parts[0] == 'offline_runs':
        capture = read_json(root/'evidence/offline_runs'/parts[1]/'job.json')['session']
    elif not capture and len(parts) >= 2 and parts[0] == 'reconstruction':
        capture = parts[1]
    if not CAPTURE_ID.fullmatch(capture):
        return []
    folder = root/'evidence/reconstruction'/capture
    files = [p for name in ('frames.jsonl','unposed_frames.jsonl')
             if (p := folder/name).is_file() and p.stat().st_size > 0]
    return files or sorted(folder.glob('frame_*.json'))


@lru_cache(maxsize=24)
def frame_index(files):
    rows = []
    for path, _, _ in files:
        with path.open('rb') as stream:
            while True:
                offset = stream.tell()
                data = stream.read() if path.suffix == '.json' else stream.readline()
                if not data:
                    break
                record = json.loads(data)
                depth = record.get('depth') or record
                camera = 'depth_zlib_b64' in depth
                if not camera and 'points_xyz' not in record:
                    continue
                posed = 'translation' in depth if camera else True
                rows.append((depth.get('stamp',record['stamp']),path,offset,camera,posed))
    return sorted(rows)


def work_rows(root, work):
    files = [p.resolve() for p in archive_files(root,work) if p.is_file()]
    return frame_index(tuple((p,p.stat().st_size,p.stat().st_mtime_ns) for p in files
                             if p.is_relative_to((root/'evidence').resolve())))


def frame_manifest(root, work_id):
    work = saved_work(root, work_id)
    return {'title':work['title'], 'date':work['date'], 'has_preview':work['kind'] in ('points','mesh'),
            'frames':[{'stamp':stamp,'camera':camera,'posed':posed}
                      for stamp,_,_,camera,posed in work_rows(root,work)]}


def frame_points(root, work_id, number):
    rows = work_rows(root,saved_work(root,work_id))
    if not 1 <= number <= len(rows):
        raise ValueError('帧号超出范围')
    _, path, offset, camera, _ = rows[number-1]
    with path.open('rb') as stream:
        stream.seek(offset)
        record = json.loads(stream.read() if path.suffix=='.json' else stream.readline())
    points = (project_camera_depth(record.get('depth') or record,stride=1)
              if camera else np.asarray(record['points_xyz']).reshape(-1,3))
    return points.astype('<f4').tobytes()
