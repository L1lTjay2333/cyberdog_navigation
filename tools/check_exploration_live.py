"""Explicit operator-approved short test, or a nonmoving probe (default)."""
import json,time,math,argparse,uuid
from urllib.request import Request,urlopen
from urllib.error import HTTPError
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
URL='http://127.0.0.1:8765'
def call(path,data=None):
    request=Request(URL+path,data=json.dumps(data).encode() if data is not None else None,
                    headers={'Origin':URL,'Content-Type':'application/json'})
    try:
        with urlopen(request,timeout=5) as response:return json.load(response)
    except HTTPError as exc:
        raise RuntimeError(json.loads(exc.read()).get('message','网页拒绝请求')) from None
def run(execute=False,scan=False):
    owner='approved_short_test_'+uuid.uuid4().hex
    state=call('/state');target=None
    if execute or scan:
        b=state.get('battery') or {}
        if b.get('wired_charging') or b.get('wireless_charging'):raise RuntimeError('正在充电，不能进行运动测试')
    if execute:
        pose=state['pose'];target={'x':pose['x']+.30*math.cos(pose['yaw']),'y':pose['y']+.30*math.sin(pose['yaw'])}
        report=call('/exploration/target',target)['exploration']
        if report['status']!='GOAL_ROUTE':raise RuntimeError('短测试目标没有完整已知可通行路线：'+report['message'])
    events=[];seen=set();motion_started=None;deadline=time.monotonic()+(48 if scan else 22);started=False
    def collect(rows):
        for event in rows:
            key=json.dumps(event,sort_keys=True)
            if key not in seen:seen.add(key);events.append(event)
    try:
        call('/exploration/scan' if scan else '/exploration/start' if execute else '/exploration/probe',{'owner':owner,'approved':execute or scan})
        started=True
        while time.monotonic()<deadline:
            current=call('/state')['exploration']['executor']
            rows=current.get('events',[])
            collect(rows)
            if not current['active']:break
            if execute and any(e.get('event')=='CONTROL' and any(abs(v)>1e-6 for v in e.get('velocity',[])) for e in rows):
                if motion_started is None:motion_started=time.monotonic()
                if time.monotonic()-motion_started>4:call('/exploration/stop',{});break
            try:call('/exploration/heartbeat',{'owner':owner})
            except RuntimeError:
                current=call('/state')['exploration']['executor'];collect(current.get('events',[]))
                if not current['active']:break
                raise
            time.sleep(.25)
    finally:
        if started:
            call('/exploration/stop',{})
            until=time.monotonic()+5
            while time.monotonic()<until:
                current=call('/state')['exploration']['executor'];collect(current.get('events',[]))
                if not current['active']:break
                time.sleep(.2)
    result={'mode':'approved_in_place_scan' if scan else 'approved_0.30m_short_test' if execute else 'nonmoving_probe',
        'ready':any(e.get('event')=='READY' for e in events),'arrival_reported':any(e.get('event')=='ARRIVED' for e in events),
        'stop_sent':any(e.get('event')=='STOP_SENT' for e in events),'events':events,'target':target,
        'physical_acceptance':'PENDING_OPERATOR_CONFIRMATION'}
    (ROOT/'evidence'/('exploration_live_scan.json' if scan else 'exploration_live_short_test.json' if execute else 'exploration_live_probe.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({**result,'events':events[-3:],'captured_event_count':len(events)},ensure_ascii=False))
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--execute-approved',action='store_true',help='Requires CURRENT explicit field authorization; max 0.30 m goal / 4 seconds movement')
    parser.add_argument('--scan-approved',action='store_true',help='CURRENT explicit authorization for in-place .22 rad/s scan, 42 seconds maximum')
    args=parser.parse_args()
    if args.scan_approved and args.execute_approved:parser.error('Select one motion test')
    run(args.execute_approved,args.scan_approved)
