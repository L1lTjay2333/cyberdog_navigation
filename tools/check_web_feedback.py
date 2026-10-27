"""Observe web feedback; optional disarmed camera probe, NEVER send motion."""
import argparse
import json
import time
import uuid
from pathlib import Path
from urllib.request import Request,urlopen

BASE='http://127.0.0.1:8765'
ROOT=Path(__file__).resolve().parents[1]
def call(path,body=None):
    request=Request(BASE+path,headers={'Origin':BASE,'Content-Type':'application/json'},
                    data=None if body is None else json.dumps(body).encode())
    with urlopen(request,timeout=8) as response:return json.load(response)

def run(seconds=15,manual_probe=False,localization_probe=False):
    owner='readonly_probe_'+uuid.uuid4().hex;connected=False;rows=[]
    initial=call('/state');manual=call('/manual_state')
    if manual_probe or localization_probe:
        if manual.get('connected') or initial['navigation']['active'] or initial['mapping']['active'] or initial['exploration']['executor']['active']:
            raise RuntimeError('已有会话，不抢占控制')
    if localization_probe:
        call('/relocalize',{}) # Localization action only; never a navigation goal.
        deadline=time.monotonic()+90
        while True:
            s=call('/state');n=s['navigation']
            if not n['active']:
                if not any(e['event']=='POSITION_REFRESHED' for e in n['events']):
                    raise RuntimeError('不运动定位检查未完成：'+str(n['events'][-2:]))
                break
            if time.monotonic()>deadline:
                call('/cancel',{});raise RuntimeError('不运动定位检查超时，已请求取消，不自动重试')
            time.sleep(.25)
    if manual_probe:
        call('/manual/connect',{'owner':owner});connected=True
    try:
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            s=call('/state');m=call('/manual_state?owner='+owner) if connected else {}
            now=time.time();pose=s.get('pose') or {}
            rows.append({'unix':now,'connection':s.get('connection'),'task':(s.get('task') or {}).get('status'),
                'stream_age_s':now-s.get('emitted_unix',0),'pose_frame':pose.get('frame'),
                'pose_age_s':now-pose['received_unix'] if pose else None,
                'manual_connected':m.get('connected'),'manual_active':m.get('active'),
                'depth_valid':m.get('depth_valid'),'depth_age_s':m.get('depth_age_s'),
                'depth_frame_age_s':m.get('depth_frame_age_s'),'depth_source':m.get('depth_source'),
                'depth_invalid_reason':m.get('depth_invalid_reason'),'forward_block':(m.get('direction_blocks') or {}).get('forward'),
                'battery':s.get('battery')})
            time.sleep(.25)
    finally:
        if connected:call('/manual/disconnect',{'owner':owner})
    summary={'mode':'LOCALIZATION_AND_DISARMED_CAMERA_FEEDBACK' if localization_probe else 'DISARMED_CAMERA_FEEDBACK' if manual_probe else 'PASSIVE_FEEDBACK',
        'motion_requests_sent':0,'samples':len(rows),'stream_age_max_s':max(r['stream_age_s'] for r in rows),
        'stream_age_mean_s':sum(r['stream_age_s'] for r in rows)/len(rows),
        'connected_samples':sum(r['connection']=='connected' for r in rows),
        'depth_valid_samples':sum(r['depth_valid'] is True for r in rows),
        'forward_blocks':sorted(set(r['forward_block'] for r in rows if r['forward_block'])),
        'position_frames':sorted(set(r['pose_frame'] for r in rows if r['pose_frame'])),
        'localized_samples':sum(r['task']==8 and r['pose_frame']=='map' and r['pose_age_s'] is not None and r['pose_age_s']<=2 for r in rows),
        'physical_navigation_acceptance':'NOT_TESTED'}
    (ROOT/'evidence/web_feedback_stability_20261010.json').write_text(json.dumps({'summary':summary,'samples':rows},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--manual-probe',action='store_true');parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--localization-probe',action='store_true',help='Explicit nonmoving stock localization, no navigation')
    args=parser.parse_args();run(args.seconds,args.manual_probe,args.localization_probe)
