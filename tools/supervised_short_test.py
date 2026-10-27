"""One explicitly authorized short navigation test through the web API."""
import json
import math
import time
import urllib.request
import argparse
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
scope_path=ROOT/'config/scope.json'
evidence=ROOT/('evidence/short_test_'+time.strftime('%Y%m%d')+'.jsonl')
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--forward-distance',type=float,default=None,
                    help='Explicit supervised target distance, 0.5 to 1.2 metres; still cancel at 0.2m displacement')
args=parser.parse_args()
if args.forward_distance is not None and not .5<=args.forward_distance<=1.2:
    parser.error('forward-distance must be between 0.5 and 1.2 metres')

def log(**row):
    row['unix']=time.time()
    text=json.dumps(row,ensure_ascii=False)
    with evidence.open('a',encoding='utf-8') as f:f.write(text+'\n')
    print(text,flush=True)

def get():
    return json.loads(urllib.request.urlopen('http://127.0.0.1:8765/state',timeout=4).read())

def post(path,body):
    request=urllib.request.Request('http://127.0.0.1:8765'+path,data=json.dumps(body).encode(),
        headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8765'})
    try:
        response=json.loads(urllib.request.urlopen(request,timeout=30).read())
    except urllib.error.HTTPError as exc:
        raise RuntimeError(exc.read().decode('utf-8')) from exc
    if not response.get('ok'):raise RuntimeError(response)
    return response

scope=json.loads(scope_path.read_text(encoding='utf-8'))
sent=False
cancelled=False
try:
    s=get();p=s.get('pose');g=s.get('map')
    if s.get('navigation',{}).get('active'):raise RuntimeError('Existing session; abort')
    if s.get('task',{}).get('status')!=8 or not p or not g or g['frame']!='map':
        raise RuntimeError('Localization not ready')
    if time.time()-s.get('local_received_unix',0)>3 or s['emitted_unix']-p['received_unix']>2:
        raise RuntimeError('Stale robot position')
    r=g['resolution'];o=g['origin'];a=2*math.atan2(o['qz'],o['qw']);candidates=[]
    for iy in range(5,g['height']-5):
        for ix in range(5,g['width']-5):
            lx=(ix+.5)*r;ly=(iy+.5)*r
            x=o['x']+math.cos(a)*lx-math.sin(a)*ly;y=o['y']+math.sin(a)*lx+math.cos(a)*ly
            dx=x-p['x'];dy=y-p['y'];d=math.hypot(dx,dy)
            turn=abs(math.atan2(math.sin(math.atan2(dy,dx)-p['yaw']),math.cos(math.atan2(dy,dx)-p['yaw'])))
            if .5<d<.85 and turn<math.pi/3 and all(g['data'][(iy+dyi)*g['width']+ix+dxi]==0 for dyi in range(-5,6) for dxi in range(-5,6)):
                candidates.append((abs(d-.6)+turn*.3,x,y))
    if args.forward_distance is not None:
        x=p['x']+args.forward_distance*math.cos(p['yaw'])
        y=p['y']+args.forward_distance*math.sin(p['yaw'])
        dx=x-o['x'];dy=y-o['y']
        ix=math.floor((math.cos(a)*dx+math.sin(a)*dy)/r)
        iy=math.floor((-math.sin(a)*dx+math.cos(a)*dy)/r)
        if not (5<=ix<g['width']-5 and 5<=iy<g['height']-5) or not all(g['data'][(iy+dyi)*g['width']+ix+dxi]==0 for dyi in range(-5,6) for dxi in range(-5,6)):
            raise RuntimeError('Explicit target fails original clearance check; no motion sent')
        candidates=[(0,x,y)]
    if not candidates:raise RuntimeError('No short free target ahead; no motion sent')
    _,x,y=min(candidates);target={'x':x,'y':y}
    route=post('/goal',target)
    route_limit=2.0 if args.forward_distance is not None else 1.2
    if route['preview']['length_m']>route_limit:raise RuntimeError('Preview route too long; no motion sent')
    if args.forward_distance is not None:
        for point in route['preview']['points']:
            dx=point['x']-p['x'];dy=point['y']-p['y']
            forward=dx*math.cos(p['yaw'])+dy*math.sin(p['yaw'])
            lateral=-dx*math.sin(p['yaw'])+dy*math.cos(p['yaw'])
            if not -.05<=forward<=1.5 or abs(lateral)>.65:
                raise RuntimeError('Preview leaves confirmed clear test corridor; no motion sent')
    log(event='PREVIEW',start=p,goal=target,length_m=route['preview']['length_m'])
    enabled=dict(scope);enabled['motion_enabled']=True;enabled['motion_max_goal_distance_m']=args.forward_distance+.1 if args.forward_distance is not None else .9
    scope_path.write_text(json.dumps(enabled,indent=2),encoding='utf-8')
    # Mark uncertainty before POST: never resend if the acknowledgement is lost.
    sent=True
    post('/navigate',target)
    log(event='WEB_SUBMITTED_ONCE')
    started=time.monotonic();ack_at=None;previous_events=0;last_log=0;max_displacement=0
    while time.monotonic()-started<240:
        s=get();n=s['navigation'];events=n['events'];now=time.monotonic()
        for row in events[previous_events:]:
            log(event='ROBOT_EVENT',data=row)
            if row.get('event')=='ACK' and row.get('accepted'):
                ack_at=now
                if s.get('pose') and s['pose'].get('frame')=='map' and s['emitted_unix']-s['pose']['received_unix']<2:p=dict(s['pose'])
        previous_events=len(events)
        current=s.get('pose');fresh=current and current.get('frame')=='map' and s['emitted_unix']-current['received_unix']<2
        displacement=math.hypot(current['x']-p['x'],current['y']-p['y']) if fresh else None
        if displacement is not None:max_displacement=max(max_displacement,displacement)
        if now-last_log>3:
            log(event='MONITOR',active=n['active'],task=s.get('task'),pose=current,displacement_m=displacement)
            last_log=now
        if not n['active']:
            log(event='SESSION_END',max_estimated_displacement_m=max_displacement,task=s.get('task'),pose=current)
            break
        should_cancel=(ack_at is not None and ((displacement is not None and displacement>=.2) or now-ack_at>20 or not fresh)) or (ack_at is None and now-started>180)
        if should_cancel and not cancelled:
            cancelled=True;post('/cancel',{});log(event='CANCEL_ONCE',displacement_m=displacement)
        time.sleep(.5)
    else:
        if not cancelled:post('/cancel',{});cancelled=True
        log(event='MONITOR_TIMEOUT',note='Stop result not confirmed')
except Exception as exc:
    log(event='TEST_ERROR',message=str(exc))
    if sent and not cancelled:
        try:post('/cancel',{});log(event='CANCEL_AFTER_ERROR')
        except Exception as stop_exc:log(event='CANCEL_CONNECTION_ERROR',message=str(stop_exc))
finally:
    scope['motion_enabled']=False
    scope_path.write_text(json.dumps(scope,indent=2),encoding='utf-8')
    log(event='MOTION_LOCKED')
