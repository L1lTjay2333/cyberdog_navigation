"""Display-only measured trajectory; never used as a navigation path."""
import math
from collections import deque
from telemetry_time import sensor_age

class MotionTrail:
    def __init__(self):
        self.key=None;self.points=deque(maxlen=6000);self.previous=None;self.distance=0.

    def update(self,snapshot,session,now):
        pose=snapshot.get('pose') or {};grid=snapshot.get('map') or {}
        frame=grid.get('frame');key=(snapshot.get('started_unix'),session,frame)
        if key!=self.key:
            self.key=key;self.points.clear();self.previous=None;self.distance=0.
        valid=(snapshot.get('connection')=='connected' and pose.get('frame')==frame
               and sensor_age(snapshot,pose.get('received_unix'),now)<2
               and all(type(pose.get(k)) in (float,int) and math.isfinite(pose[k]) for k in ('x','y'))
               and ((snapshot.get('task') or {}).get('status')==5 or
                    not (snapshot.get('localization_validation') or {}).get('required',True)))
        if valid:
            point={k:pose[k] for k in ('x','y','received_unix')}
            previous=self.previous
            if previous is None:
                point['break']=True;self.points.append(point);self.previous=point
            else:
                dt=point['received_unix']-previous['received_unix']
                distance=math.hypot(point['x']-previous['x'],point['y']-previous['y'])
                if dt>0 and distance>=.03:
                    point['break']=dt>2 or distance>max(.4,1.5*dt)
                    if not point['break']:self.distance+=distance
                    self.points.append(point);self.previous=point
                elif dt>0 and distance<.03:
                    # Refresh time without drawing stationary sensor jitter.
                    self.previous={**previous,'received_unix':point['received_unix']}
        else:self.previous=None
        return {'frame':frame,'points':list(self.points),'distance_m':self.distance,'kind':'measured'}
