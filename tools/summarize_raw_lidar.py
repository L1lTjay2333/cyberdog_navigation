"""Summarize the recorded SDK and stock ROS scans without motion."""
import json,math,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
rows=[json.loads(s) for s in (ROOT/'evidence/raw_lidar_comparison_20261008.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
stages={'ROS_BEFORE':[],'SDK':[],'ROS_AFTER':[]}
sdktext=''
for row in rows:
 if row['event'] in ('ROS_BEFORE','ROS_AFTER'):stages[row['event']].append(row)
 if row['event']=='SDK_FINISH':
  sdktext=row['stdout']
  for line in sdktext.splitlines():
   if line.startswith('RAW_JSON '):stages['SDK'].append(json.loads(line[len('RAW_JSON '):]))
result={}
for stage,frames in stages.items():
 summaries=[]
 for frame in frames:
  pts=[p for p in frame['points'] if p[0] is not None and p[1] is not None and .01<=p[1]<=30]
  central=[p[1] for p in pts if abs(math.atan2(math.sin(p[0]),math.cos(p[0])))<=math.radians(15)]
  summaries.append({'points':len(pts),'central_count':len(central),'central_min_m':min(central) if central else None,
                    'central_median_m':statistics.median(central) if central else None,
                    'central_below_0_6m':sum(v<.6 for v in central)})
 values=[r['central_median_m'] for r in summaries if r['central_median_m'] is not None]
 result[stage]={'frames':len(frames),'median_of_frame_medians_m':statistics.median(values) if values else None,'per_frame':summaries}
result['restored']=any(r['event']=='RESTORE_CHECK' and r.get('restored') for r in rows)
result['note']='SDK versus ROS timing is sequential, not simultaneous; no target identity or accuracy guarantee'
(ROOT/'evidence/raw_lidar_comparison_summary_20261008.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
print('SDK startup metadata (serial identifiers omitted):')
for line in sdktext.splitlines():
 if any(k in line.lower() for k in ('model','sampling','scan frequency','firmware','hardware')) and 'serial' not in line.lower():print(line)
