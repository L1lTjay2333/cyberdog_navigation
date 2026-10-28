"""Bounded, append-only PC submap storage shared by online and offline fusion."""
import json
from pathlib import Path
import numpy as np
from reconstruction_registration import cloud_array


class DiskSubmaps:
    def __init__(self, folder, voxel=.01, max_voxels=200000, max_parts=64, max_bytes=512*1024*1024):
        if not np.isfinite(voxel) or voxel <= 0 or type(max_voxels) is not int or max_voxels < 1:
            raise ValueError('Invalid submap bounds')
        if type(max_parts) is not int or max_parts < 1 or type(max_bytes) is not int or max_bytes < 1:
            raise ValueError('Invalid disk bounds')
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.voxel, self.max_voxels = voxel, max_voxels
        self.max_parts, self.max_bytes = max_parts, max_bytes
        self.cells, self.parts = {}, []
        self.previous = None
        self.frames = 0
        self.bytes = 0
        self.closed = False

    def grouped(self, points):
        points = cloud_array(points)
        if not len(points):
            raise ValueError('Empty submap input')
        scaled = np.floor(points/self.voxel)
        if np.any(abs(scaled) > np.iinfo(np.int64).max // 2):
            raise ValueError('Voxel coordinates exceed bounds')
        keys, inv, counts = np.unique(scaled.astype(np.int64), axis=0, return_inverse=True, return_counts=True)
        sums = np.column_stack([np.bincount(inv, weights=points[:, i]) for i in range(3)])
        return [(tuple(key), total, int(count)) for key, total, count in zip(keys, sums, counts)]

    @staticmethod
    def merge(cells, groups):
        for key, total, count in groups:
            cell = cells.setdefault(key, [np.zeros(3), 0, 0])
            cell[0] += total
            cell[1] += count
            cell[2] += 1

    def seal(self):
        if not self.cells:
            return
        if len(self.parts) >= self.max_parts:
            raise ValueError('Submap part limit reached; old submaps preserved')
        points = np.asarray([cell[0]/cell[1] for cell in self.cells.values()])
        support = np.asarray([cell[2] for cell in self.cells.values()], dtype=np.uint32)
        # Uncompressed NPZ size can be bounded before writing, including ZIP headers.
        upper_bytes = points.nbytes + support.nbytes + 4096
        if self.bytes + upper_bytes > self.max_bytes:
            raise ValueError('Submap disk quota reached; old submaps preserved')
        name = 'submap_%03d.npz' % len(self.parts)
        path = self.folder/name
        temporary = path.with_suffix('.tmp')
        with temporary.open('xb') as stream:
            np.savez(stream, points=points, support=support)
        size = temporary.stat().st_size
        temporary.replace(path)
        self.bytes += size
        self.parts.append({'file':name, 'voxels':len(points), 'stable_points':int((support>=2).sum()), 'bytes':size})
        self.cells = {}

    def add(self, points):
        if self.closed:
            raise ValueError('Submaps already finalized')
        groups = self.grouped(points)
        additions = sum(key not in self.cells for key, _, _ in groups)
        if len(self.cells)+additions > self.max_voxels:
            # Validate the next part, including an overlap frame, before sealing.
            candidate = {}
            if self.previous is not None:
                self.merge(candidate, self.previous)
            self.merge(candidate, groups)
            if len(candidate) > self.max_voxels:
                raise ValueError('Two overlapping frames exceed submap capacity; no frame discarded')
            self.seal()
            self.cells = candidate
        else:
            self.merge(self.cells, groups)
        self.previous = groups
        self.frames += 1

    def finish(self):
        if self.closed:
            return self.manifest
        self.seal()
        self.manifest = {'frames':self.frames, 'voxel_m':self.voxel, 'parts':self.parts,
                         'max_voxels_per_part':self.max_voxels, 'bytes':self.bytes,
                         'rollover_note':'At rollover old and candidate dictionaries coexist; per-part bound is not peak RAM.',
                         'boundary_overlap_frames':1, 'loop_closure':False,
                         'stable_point_count_is_per_part':True,
                         'note':'Boundary points can occur in two parts. This is not a global deduplicated model.'}
        (self.folder/'manifest.json').write_text(json.dumps(self.manifest, indent=2), encoding='utf-8')
        self.closed = True
        return self.manifest
