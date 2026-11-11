"""Turn existing PC capture archives into portfolio previews."""
import hashlib
import json
import math
import re
import shutil
import struct
import zlib
from array import array
from pathlib import Path

from saved_grid import load_saved_grid


def _png(path, width, height, pixels):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))

    rows = b''.join(b'\0' + pixels[i:i+width] for i in range(0, len(pixels), width))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def _read_cloud(path):
    preview = array('f')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        if stream.readline() != b'ply\n':
            raise ValueError('Invalid PLY')
        digest.update(b'ply\n')
        count = None
        for _ in range(32):
            line = stream.readline()
            digest.update(line)
            if line.startswith(b'element vertex '):
                count = int(line.split()[-1])
            if line == b'end_header\n':
                break
        else:
            raise ValueError('Incomplete PLY header')
        if count is None or count < 0:
            raise ValueError('Invalid PLY vertex count')
        stride = max(1, (count + 99999) // 100000)
        seen = 0
        for index, line in enumerate(stream):
            digest.update(line)
            if index % stride == 0:
                point = tuple(map(float, line.split()))
                if len(point) != 3 or not all(map(math.isfinite, point)):
                    raise ValueError('Invalid XYZ point')
                preview.extend(point)
            seen += 1
        if seen != count:
            raise ValueError('Incomplete PLY')
    return count, preview, digest.digest()


def _thumbnail(path, points):
    width, height = 320, 180
    pixels = bytearray([242]) * (width * height)
    if points:
        x, y = points[::3], points[1::3]
        x0, y0 = min(x), min(y)
        scale = min((width-16) / max(max(x)-x0, .001), (height-16) / max(max(y)-y0, .001))
        for px, py in zip(x, y):
            u, v = 8 + int((px-x0)*scale), height-9-int((py-y0)*scale)
            pixels[v*width+u] = 98
    _png(path, width, height, pixels)


def import_captures(root, folder, existing, removed):
    known = existing + removed
    ids = {entry['id'] for entry in known}
    # Match prior manually curated entries (including removals) against the same PLY.
    legacy_sizes = {}
    for entry in known:
        if not entry.get('capture_id'):
            asset = folder / (entry['id'] + '.ply')
            if asset.is_file():
                legacy_sizes.setdefault(asset.stat().st_size, []).append(asset)

    for session in sorted((root/'evidence/reconstruction').glob('*')):
        if not re.fullmatch(r'\d{8}_\d{6}_\d{6}', session.name):
            continue
        work_id = 'depth-' + session.name
        summary_path, ply = session/'summary.json', session/'model.ply'
        if work_id in ids or not summary_path.is_file() or not ply.is_file():
            continue
        try:
            summary = json.loads(summary_path.read_text(encoding='utf-8-sig'))
            count, points, digest = _read_cloud(ply)
            if any(hashlib.sha256(p.read_bytes()).digest() == digest for p in legacy_sizes.get(ply.stat().st_size, [])):
                continue
            archived = any(p.is_file() and p.stat().st_size > 0 for p in
                           (session/'frames.jsonl', session/'unposed_frames.jsonl', session/'frame_01.json'))
            if not count and not archived:
                continue
            _thumbnail(folder/(work_id+'.png'), points)
            if count:
                (folder/(work_id+'.bin')).write_bytes(points.tobytes())
                shutil.copyfile(ply, folder/(work_id+'.ply'))
        except (OSError, ValueError, OverflowError) as exc:
            print(f'Portfolio depth import deferred for {session.name}: {exc}', flush=True)
            continue
        stamp = session.name
        note = ('原始采集保存在电脑。' if count else '原始帧已保存，等待离线重建。')
        if summary.get('fusion_paused') or summary.get('error'):
            note += '本次采集或融合曾中断。'
        yield dict(id=work_id, capture_id=stamp, title=f'深度采集 · {stamp[9:11]}:{stamp[11:13]}:{stamp[13:15]}',
                   date=f'{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}', kind='points' if count else 'raw',
                   points=count, note=note, source=session.relative_to(root).as_posix())

    for session in sorted((root/'evidence/live_cloud_maps/saved_maps').glob('computer_room_*')):
        if not re.fullmatch(r'computer_room_\d{8}_\d{6}', session.name):
            continue
        stamp = session.name.removeprefix('computer_room_')
        work_id = 'map-' + stamp
        if work_id in ids or not (session/'manifest.json').is_file():
            continue
        try:
            grid = load_saved_grid(session, session.name)
            width, height = grid['width'], grid['height']
            data = grid['data']
            pixels = bytes({0: 246, -1: 187, 100: 38}[value] for row in range(height-1, -1, -1)
                           for value in data[row*width:(row+1)*width])
            _png(folder/(work_id+'.png'), width, height, pixels)
        except (OSError, ValueError, KeyError) as exc:
            print(f'Portfolio map import deferred for {session.name}: {exc}', flush=True)
            continue
        yield dict(id=work_id, capture_id=stamp, title=f'二维地图 · {stamp[9:11]}:{stamp[11:13]}:{stamp[13:15]}',
                   date=f'{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]}', kind='map', points=0, map=work_id+'.png',
                   note='原厂保存的二维地图和建图文件已归档到电脑。', source=session.relative_to(root).as_posix())
