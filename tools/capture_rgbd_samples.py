"""Save bounded 5-second image samples on the PC using the pinned SSH config.

No remote installation or camera start/stop. Camera streams must be active.
"""
import argparse
import base64
from datetime import datetime
import json
from pathlib import Path
import subprocess
import struct
import zlib
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def save_rgb_png(path, array):
    """Lossless 8-bit RGB PNG using the standard library."""
    h, w, channels = array.shape
    if channels != 3 or array.dtype != np.uint8:
        raise ValueError('Expected uint8 RGB image')
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff)
    pixels = b''.join(b'\x00' + row.tobytes() for row in array)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' +
        chunk(b'IHDR', struct.pack('!2I5B', w, h, 8, 2, 0, 0, 0)) +
        chunk(b'IDAT', zlib.compress(pixels)) + chunk(b'IEND', b''))


def decode_image(row):
    w, h, step = row['width'], row['height'], row['step']
    encoding = row['encoding']
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(encoding)
    dtype = np.dtype('u1' if channels else
                     ('>' if row['is_bigendian'] else '<') +
                     {'16UC1': 'u2', '32FC1': 'f4'}[encoding])
    raw = base64.b64decode(row['data'], validate=True)
    pixel_size = dtype.itemsize * (channels or 1)
    if not (0 < w <= 4096 and 0 < h <= 4096 and w * pixel_size <= step
            and len(raw) == step * h):
        raise ValueError('Invalid image layout')
    if channels:
        array = np.ndarray((h, w, channels), dtype=dtype, buffer=raw,
                           strides=(step, channels, 1))[..., :3]
        return (array[..., ::-1] if encoding.startswith('bgr') else array).copy()
    return np.ndarray((h, w), dtype=dtype, buffer=raw,
                       strides=(step, dtype.itemsize)).copy()


def save_sample(row, folder, index):
    array = decode_image(row)
    metadata = {k: v for k, v in row.items() if k != 'data'}
    name = 'sample_%03d' % index
    if array.ndim == 3:
        save_rgb_png(folder / (name + '_color.png'), array)
        metadata['color_file'] = name + '_color.png'
    else:
        np.save(folder / (name + '_depth.npy'), array)
        metadata['depth_file'] = name + '_depth.npy'
        depth = array.astype(float) * (.001 if row['encoding'] == '16UC1' else 1.)
        valid = np.isfinite(depth) & (depth >= .15) & (depth <= 4.)
        metadata['valid_depth_fraction_015_4m'] = float(valid.mean())
        infos = [c for c in row['calibration'] if c['frame_id'] == row['frame_id']
                 and c['width'] == row['width'] and c['height'] == row['height']
                 and abs(c['stamp'] - row['stamp']) <= .2]
        if len(infos) == 1:
            p = infos[0]['p']
            if len(p) != 12 or not np.isfinite(p).all() or p[0] <= 0 or p[5] <= 0 or p[3] != 0 or p[7] != 0:
                raise ValueError('Invalid depth projection')
            v, u = np.mgrid[:depth.shape[0]:4, :depth.shape[1]:4]
            z = depth[::4, ::4]; mask = valid[::4, ::4]
            xyz = np.stack(((u-p[2])*z/p[0], (v-p[6])*z/p[5], z), axis=-1)[mask]
            header = 'ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header' % len(xyz)
            np.savetxt(folder / (name + '_camera_points.ply'), xyz, fmt='%.6f', header=header, comments='')
            metadata.update(points=len(xyz), points_source='depth_backprojection',
                            points_frame=row['frame_id'])
        else:
            metadata['points_skipped'] = 'No unique same-frame, same-size, recent calibration'
    (folder / (name + '.json')).write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return metadata


def validate_saved_samples(rows, paired=False):
    """Process completion is not proof that usable RGB-D data arrived."""
    if not any('color_file' in r for r in rows) or not any(r.get('points', 0) > 0 for r in rows):
        raise ValueError('未获得完整彩色图与有效标定点云，不能报告采集成功')
    if paired:
        groups = {}
        for row in rows:
            if 'pair_id' not in row:
                raise ValueError('Missing RGB-D pair identifier')
            groups.setdefault(row['pair_id'], []).append(row)
        for pair in groups.values():
            if (len(pair) != 2 or sum('color_file' in r for r in pair) != 1
                    or sum(r.get('points', 0) > 0 for r in pair) != 1
                    or abs(pair[0]['stamp']-pair[1]['stamp']) > .05):
                raise ValueError('Incomplete, unusable or unsynchronized RGB-D pair')


def run(seconds=16, paired=False):
    if not 1 <= seconds <= 120:
        raise ValueError('Duration must be 1..120 seconds')
    folder = ROOT / 'evidence/rgbd_samples' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    source = 'CAPTURE_SECONDS=%r\nPAIR_RGBD=%r\n' % (seconds, paired) + (ROOT / 'tools/rgbd_sample_remote.py').read_text(encoding='utf-8')
    payload = base64.b64encode(zlib.compress(source.encode()))
    command = ['ssh', '-F', str(ROOT / 'config/readonly_ssh_config'), '-o', 'ServerAliveInterval=0',
        '-o', 'ConnectTimeout=20', 'cyberdog-current-nx',
        'source /etc/mi/ros2_env.conf; python3 -u -c "import sys,zlib,base64;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))"']
    rows = []; summary = None; failure = None
    try:
        result = subprocess.run(command, input=payload, capture_output=True, timeout=seconds+40)
        (folder / 'stderr.txt').write_bytes(result.stderr)
        (folder / 'raw_samples.jsonl').write_bytes(result.stdout)
        for line in result.stdout.splitlines():
            row = json.loads(line)
            if row.get('kind') == 'image':
                rows.append(save_sample(row, folder, len(rows)))
            elif row.get('kind') == 'summary':
                summary = row
        if result.returncode or summary is None:
            raise RuntimeError('Incomplete remote sample: exit %s' % result.returncode)
        validate_saved_samples(rows, paired)
    except Exception as error:
        failure = str(error)
    report = {'samples': rows, 'remote': summary, 'error': failure,
              'complete': failure is None, 'motion_requests': 0,
              'rgbd_registered': False, 'note': 'Independent camera samples; no world poses or RGB-depth registration.'}
    (folder / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'folder': str(folder), 'complete': failure is None,
        'colors': sum('color_file' in r for r in rows),
        'pointclouds': sum(r.get('points', 0) > 0 for r in rows), 'error': failure}))
    return 1 if failure else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=16)
    parser.add_argument('--paired', action='store_true', help='Select depth within 50 ms of each saved photo')
    args = parser.parse_args()
    raise SystemExit(run(args.seconds, args.paired))
