"""Live HTTP/robot feedback check; strictly never sets armed=true."""
import json
import time
import urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE='http://127.0.0.1:8765'
OWNER='offline_diagnostic_20261008'
def post(path,item):
    req=urllib.request.Request(BASE+path,data=json.dumps(item).encode(),headers={
        'Content-Type':'application/json','Origin':BASE})
    return json.load(urllib.request.urlopen(req,timeout=5))
def state():return json.load(urllib.request.urlopen(BASE+'/manual_state',timeout=5))
evidence=[]
try:
    if state().get('enabled'):raise RuntimeError('Requires motion lock false; no test sent')
    post('/manual/connect',{'owner':OWNER})
    deadline=time.monotonic()+12
    while not state().get('connected') and time.monotonic()<deadline:time.sleep(.2)
    if not state().get('connected'):raise RuntimeError('Robot feedback not connected; cannot claim live test passed')
    for seq,d in enumerate(('forward','backward','shift_left','shift_right','left','right','stop'),1):
        post('/manual/control',{'owner':OWNER,'sequence':seq,'direction':d,'armed':False})
        time.sleep(.3)
        row=state();evidence.append(row)
        if row.get('enabled') or row.get('active'):raise RuntimeError('Unexpected enabled/active state')
        if not row.get('connected'):raise RuntimeError('Robot feedback disconnected during test')
        if row.get('requested_direction')!=d:raise RuntimeError('Request not reflected')
    print('PASS: seven HTTP requests with live robot feedback; motion lock false')
finally:
    post('/manual/disconnect',{})
    (ROOT/'evidence/manual_locked_live_20261008.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
