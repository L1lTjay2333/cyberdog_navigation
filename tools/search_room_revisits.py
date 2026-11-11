"""Search separated room scan windows for bidirectional surface overlap."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]

def windows():
    groups = []
    batch = []
    index = 0
    for file in sorted((ROOT/'evidence/live_room_processed_20261103').glob('session*.jsonl')):
        with file.open(encoding='utf-8') as source:
            for line in source:
                row = json.loads(line)
                batch.append(row['points_xyz'])
                index += 1
                if len(batch) == 5:
                    points = np.concatenate(batch)
                    _, representatives = np.unique(np.floor(points/.08), axis=0, return_index=True)
                    groups.append((index-4, points[representatives]))
                    batch.clear()
    return groups

def run():
    groups = windows()
    count = len(groups)//5
    results = []
    for i, first in groups[:count]:
        tree = cKDTree(first)
        for j, last in groups[-count:]:
            forward = cKDTree(last).query(first, distance_upper_bound=.1)[0]
            reverse = tree.query(last, distance_upper_bound=.1)[0]
            fractions = [float(np.isfinite(d).mean()) for d in [forward, reverse]]
            results.append(dict(first=i,last=j,bidirectional=min(fractions),overlap=fractions))
    results.sort(key=lambda r:r['bidirectional'],reverse=True)
    report = dict(window_frames=5,voxel_m=.08,radius_m=.1,pairs=len(results),best=results[:10],
                  endpoint=next(r for r in results if r['first']==1 and r['last']==groups[-1][0]),
                  correction_applied=False)
    dest=ROOT/'evidence/room_registration_replay/revisit_search.json'
    dest.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps([report['pairs'],report['best'][0],report['endpoint']]))
if __name__=='__main__':
    run()
