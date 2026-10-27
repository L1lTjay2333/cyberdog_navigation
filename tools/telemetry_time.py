"""Compare sensor timestamps in the robot clock, receipt age in the PC clock."""
import math

def sensor_age(snapshot, stamp, now):
    emitted=snapshot.get('emitted_unix',now)
    receipt=snapshot.get('local_received_unix',now)
    values=(emitted,receipt,stamp,now)
    if not all(type(v) in (int,float) and math.isfinite(v) for v in values):return math.inf
    if emitted-stamp < -.5:return math.inf
    return max(0,emitted-stamp)+max(0,now-receipt)

def event_local_time(snapshot,event):
    return event.get('local_received_unix',event.get('unix',0)+snapshot.get('local_received_unix',0)-snapshot.get('emitted_unix',0))
