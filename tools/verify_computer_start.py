"""Single explicit non-walking localization test; never navigate."""
import json
import time
import urllib.request
from pathlib import Path

BASE='http://127.0.0.1:8765'
OUTPUT=Path(__file__).resolve().parents[1]/('evidence/COMPUTER_START_TEST_'+time.strftime('%Y%m%d')+'.jsonl')
def get():return json.loads(urllib.request.urlopen(BASE+'/state',timeout=4).read())
def post(path):
    req=urllib.request.Request(BASE+path,data=b'{}',headers={'Content-Type':'application/json','Origin':BASE})
    return json.loads(urllib.request.urlopen(req,timeout=10).read())
def log(row):
    text=json.dumps(row,ensure_ascii=False)
    with OUTPUT.open('a',encoding='utf-8') as f:f.write(text+'\n')
    print(text,flush=True)
if __name__=='__main__':
    s=get()
    if s['navigation']['enabled'] or s['navigation']['active']:raise SystemExit('Motion open or session busy; abort')
    log({'submitted_once':post('/relocalize')})
    started=time.monotonic();seen=0;last=0
    while time.monotonic()-started<230:
        s=get();n=s['navigation']
        for e in n['events'][seen:]:log(e)
        seen=len(n['events'])
        if time.monotonic()-last>10:
            log({'active':n['active'],'task':s.get('task'),'decision':s.get('decision')});last=time.monotonic()
        if not n['active']:
            c=s.get('cloud') or {}
            log({'finished':True,'decision':s['decision'],'pose':s.get('pose'),'cloud_frame':c.get('frame'),'cloud_points':len(c.get('points_xyz',[])),'motion_enabled':n['enabled']})
            break
        time.sleep(.4)
    else:
        post('/cancel');log({'timeout':True,'cancel_requested':True})
