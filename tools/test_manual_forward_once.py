"""Explicitly supervised 1-second forward trial. Restore locks in finally."""
import json,time,urllib.request
from pathlib import Path
from manual_bridge import ManualBridge
ROOT=Path(__file__).resolve().parents[1]
scope_path=ROOT/'config/scope.json'
scope=json.loads(scope_path.read_text(encoding='utf-8'))
if scope.get('manual_motion_enabled') or scope.get('motion_enabled'):
    raise RuntimeError('Trial requires initially locked configuration')
snapshot=json.load(urllib.request.urlopen('http://127.0.0.1:8765/state',timeout=5))
manual=json.load(urllib.request.urlopen('http://127.0.0.1:8765/manual_state',timeout=5))
if snapshot.get('navigation',{}).get('active') or snapshot.get('mapping',{}).get('active') or manual.get('connected'):
    raise RuntimeError('Other computer session present; no test sent')
b=ManualBridge(ROOT);rows=[];owner='supervised_forward_once_20261008'
def log(event,**data):
    row=dict(event=event,unix=time.time(),**data);rows.append(row);print(json.dumps(row),flush=True)
try:
    enabled=dict(scope);enabled['manual_motion_enabled']=True
    scope_path.write_text(json.dumps(enabled,indent=2),encoding='utf-8')
    b.connect(owner)
    deadline=time.monotonic()+15;stable=0
    while time.monotonic()<deadline:
        s=b.report()
        ready=s.get('connected') and s.get('depth_valid') and not s.get('reason')
        ready=ready and (s.get('motion_status') or {}).get('motion_id')==112
        stable=stable+1 if ready else 0
        if stable>=3:break
        time.sleep(.25)
    if stable<3:
        log('BLOCKED',state=b.report());raise RuntimeError('No stable protection readiness; no motion requested')
    log('READY',state=b.report())
    begun=time.monotonic();seq=0
    while time.monotonic()-begun<1.:
        seq+=1;b.control(dict(owner=owner,sequence=seq,direction='forward',armed=True))
        s=b.report();log('FORWARD_WINDOW',state=s)
        if not s.get('connected') or s.get('reason'):break
        time.sleep(.1)
    b.control(dict(owner=owner,sequence=seq+1,direction='stop',armed=False))
    time.sleep(.5)
    log('STOP_FEEDBACK',state=b.report(),physical_stop_confirmation='PENDING_USER')
finally:
    b.disconnect()
    scope_path.write_text(json.dumps(scope,indent=2),encoding='utf-8')
    (ROOT/'evidence/manual_forward_once_20261008.jsonl').write_text(
        '\n'.join(json.dumps(r) for r in rows)+'\n',encoding='utf-8')
    print('Motion locks restored; no automatic retry',flush=True)
