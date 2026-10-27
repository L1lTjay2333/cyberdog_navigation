"""Owned web teleoperation test, not map navigation or obstacle acceptance.

Default connects and checks readiness without motion. Explicit approval permits
at most two seconds at 0.08 m/s, followed by stop/disconnect, never auto-retry.
"""
import argparse,json,time,uuid,math
from pathlib import Path
from check_exploration_live import call
ROOT=Path(__file__).resolve().parents[1]

def run(execute=False):
    state=call('/state');manual=call('/manual_state')
    if manual.get('connected') or state['navigation']['active'] or state['mapping']['active'] or state['exploration']['executor']['active']:
        raise RuntimeError('已有运动或任务会话，不争抢控制')
    b=state.get('battery') or {}
    if b.get('wired_charging') or b.get('wireless_charging'):
        raise RuntimeError('正在充电，不允许运动测试')
    owner='approved_manual_'+uuid.uuid4().hex;sequence=0;rows=[];started=False
    def record(event,**data):
        rows.append(dict(event=event,unix=time.time(),**data))
    def command(direction,armed,axes=None):
        nonlocal sequence
        sequence+=1
        return call('/manual/control',dict(owner=owner,sequence=sequence,direction=direction,armed=armed,axes=axes))
    try:
        call('/manual/connect',{'owner':owner});started=True
        until=time.monotonic()+15;stable=0
        while time.monotonic()<until:
            m=call('/manual_state?owner='+owner)
            ready=m.get('connected') and m.get('enabled') and not m.get('direction_blocks',{}).get('forward') and not m.get('reason')
            stable=stable+1 if ready else 0
            if stable>=3:break
            time.sleep(.15)
        if stable<3:record('NOT_READY',state=m);raise RuntimeError('前进数据未稳定就绪，没有发送运动')
        record('READY',state=m);origin=state.get('odom')
        if execute:
            until=time.monotonic()+2
            while time.monotonic()<until:
                command('forward',True,[.16,0]) # 0.5 * 0.16 = 0.08 m/s.
                m=call('/manual_state?owner='+owner);record('CONTROL',state=m)
                if not m.get('connected') or m.get('reason'):break
                time.sleep(.08)
        command('stop',False);time.sleep(.3)
        record('STOP_FEEDBACK',state=call('/manual_state?owner='+owner))
        final=call('/state').get('odom')
        if origin and final and origin.get('frame')==final.get('frame'):
            record('ODOMETRY_DELTA',distance_m=math.hypot(final['x']-origin['x'],final['y']-origin['y']),source='measured_odom_not_physical_acceptance')
    except Exception as exc:
        record('ERROR',message=str(exc));raise
    finally:
        if started:
            call('/manual/disconnect',{'owner':owner})
            until=time.monotonic()+4
            while time.monotonic()<until:
                if not call('/manual_state').get('connected'):break
                time.sleep(.1)
        result={'mode':'approved_low_speed_manual' if execute else 'nonmoving_manual_probe','events':rows,'physical_acceptance':'PENDING_OPERATOR_CONFIRMATION'}
        (ROOT/'evidence/low_speed_manual_test.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({**result,'events':[dict(event=r['event'],unix=r['unix'],active=(r.get('state') or {}).get('active'),reason=(r.get('state') or {}).get('reason'),distance_m=r.get('distance_m'),message=r.get('message')) for r in rows]},ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--execute-approved',action='store_true');args=p.parse_args();run(args.execute_approved)
