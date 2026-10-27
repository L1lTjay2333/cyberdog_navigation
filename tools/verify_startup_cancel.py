"""One explicitly approved startup/cancel trial. Never retries a goal."""
import json
import math
import time
import urllib.request
import urllib.error
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
BASE='http://127.0.0.1:8765'
def get():
    with urllib.request.urlopen(BASE+'/state',timeout=3) as r:return json.load(r)
def post(path,data):
    req=urllib.request.Request(BASE+path,json.dumps(data).encode(),
        {'Origin':BASE,'Content-Type':'application/json'},method='POST')
    try:
        with urllib.request.urlopen(req,timeout=30) as r:return json.load(r)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(exc.read().decode('utf-8')) from exc

if __name__=='__main__':
    import sys
    if '--execute-approved-startup-cancel' not in sys.argv:raise SystemExit('Explicit real-robot test flag required')
    record={'kind':'REAL_STARTUP_CANCEL','started_unix':time.time(),'samples':[],'physical_confirmation':'PENDING'}
    started=False;cancelled=False
    try:
        s=get();p=s['pose']
        if s['task']['status']!=8 or p['frame']!='map' or s['navigation']['active']:raise RuntimeError('Not ready')
        # A new short goal only; never reuse the earlier obstacle route.
        goal={'x':p['x']+.3*math.cos(p['yaw']),'y':p['y']+.3*math.sin(p['yaw'])}
        record['goal']=goal
        post('/navigate',goal);started=True
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            s=get();events=s['navigation']['events']
            record['samples'].append({k:s.get(k) for k in ('local_received_unix','task','pose','odom','navigation','decision')})
            if not cancelled and any(e['event']=='ACK' and e.get('accepted') for e in events):
                post('/cancel',{});cancelled=True;record['cancel_requested_unix']=time.time()
            if not s['navigation']['active']:
                if not cancelled:raise RuntimeError('Startup failed before cancellation; no cancellation acceptance')
                record['task_terminal']=any(e['event']=='STOP_TASK_TERMINAL' for e in events)
                break
            time.sleep(.15)
        else:raise RuntimeError('Cancellation trial timed out; require physical emergency stop')
        # Observe delayed startup for 15 seconds after client completion.
        for _ in range(30):
            s=get();record['samples'].append({k:s.get(k) for k in ('local_received_unix','task','pose','odom','decision')});time.sleep(.5)
        poses=[row['odom'] for row in record['samples'] if row.get('odom')]
        record['odom_max_displacement_m']=max((math.hypot(p['x']-poses[0]['x'],p['y']-poses[0]['y']) for p in poses),default=None)
        record['result']='TASK_TERMINAL_FIELD_CONFIRMATION_REQUIRED' if record['task_terminal'] else 'STOP_UNCONFIRMED'
    except Exception as exc:
        record['result']='FAILED';record['error']=str(exc)
    finally:
        if started and not cancelled:
            try:post('/cancel',{})
            except Exception:pass
        record['ended_unix']=time.time()
        (ROOT/'evidence/startup_cancel_20261009.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in record.items() if k!='samples'},ensure_ascii=False))
