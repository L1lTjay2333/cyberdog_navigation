"""Track relocation invalidation separately from fresh ROS timestamps."""
import math
import threading
import time
from telemetry_time import sensor_age,event_local_time


class LocalizationValidity:
    def __init__(self,rejected_map_name=None):
        self.lock=threading.Lock()
        self.required=True
        self.reason='电脑会话已启动，请先重新定位确认当前场地'
        self.invalid_since=time.time()
        self.map_key=None
        self.stream=None
        self.last_receipt=None
        self.odom=None
        self.charging=False
        self.verified_event=None
        self.rejected_map_name=rejected_map_name

    def invalidate(self,reason,now=None):
        with self.lock:self._invalidate(reason,time.time() if now is None else now)

    def _invalidate(self,reason,now):
        self.required=True;self.reason=reason;self.invalid_since=now

    def update(self,snapshot,transport,map_name,now=None):
        now=time.time() if now is None else now
        with self.lock:
            grid=snapshot.get('map') or {};origin=grid.get('origin') or {}
            key=(map_name,grid.get('frame'),grid.get('width'),grid.get('height'),grid.get('resolution'),
                 origin.get('x'),origin.get('y'),origin.get('qz'),origin.get('qw'))
            if grid.get('frame')=='map':
                if self.map_key is not None and key!=self.map_key:
                    self._invalidate('当前地图已切换，请重新定位；新场地需先扫描并保存',now)
                self.map_key=key
            receipt=snapshot.get('local_received_unix',0)
            fresh=snapshot.get('connection')=='connected' and 0<=now-receipt<3
            if fresh:
                stream=snapshot.get('started_unix')
                if self.stream is not None and stream is not None and stream!=self.stream:
                    self._invalidate('数据连接已重建，请重新定位确认搬动后的实际位置',now)
                if stream is not None:self.stream=stream
                if self.last_receipt is not None and receipt-self.last_receipt>6:
                    self._invalidate('机器人曾断开连接，请重新定位确认当前场地',now)
                self.last_receipt=receipt
                battery=snapshot.get('battery') or {}
                if sensor_age(snapshot,battery.get('received_unix',0),now)<3:
                    charging=bool(battery.get('wired_charging') or battery.get('wireless_charging'))
                    if charging and not self.charging:
                        self._invalidate('充电后位置可能改变，拔线放稳后需要重新定位',now)
                    if self.charging and not charging:
                        self._invalidate('充电已结束，请放稳后重新定位；不沿用充电期间的位置',now)
                    self.charging=charging
                odom=snapshot.get('odom') or {}
                if all(type(odom.get(k)) in (int,float) and math.isfinite(odom[k]) for k in ('x','y','yaw','received_unix')):
                    previous=self.odom
                    if previous and previous.get('frame')==odom.get('frame'):
                        dt=odom['received_unix']-previous['received_unix']
                        turn=abs(math.atan2(math.sin(odom['yaw']-previous['yaw']),math.cos(odom['yaw']-previous['yaw'])))
                        if 0<dt<=.5 and (math.hypot(odom['x']-previous['x'],odom['y']-previous['y'])>.6 or turn>.8):
                            self._invalidate('里程计出现搬动或重置跳变，请重新定位',now)
                    self.odom=dict(odom)
            events=transport.get('events') or []
            event=next((e for e in reversed(events) if e.get('event')=='POSITION_REFRESHED'),None)
            pose=snapshot.get('pose') or {}
            if (fresh and not self.charging and event and event_local_time(snapshot,event)>=self.invalid_since
                    and event.get('unix')!=self.verified_event and pose.get('frame')=='map'
                    and (snapshot.get('task') or {}).get('status')==8
                    and sensor_age(snapshot,pose.get('received_unix',0),now)<2):
                if all(type(p.get(k)) in (int,float) and math.isfinite(p[k]) for p in (pose,event) for k in ('x','y','yaw')):
                    turn=abs(math.atan2(math.sin(pose['yaw']-event['yaw']),math.cos(pose['yaw']-event['yaw'])))
                    if math.hypot(pose['x']-event['x'],pose['y']-event['y'])<=.2 and turn<=math.radians(10):
                        self.required=False;self.reason='本次重新定位已完成';self.verified_event=event['unix']
            # A field-confirmed false match is stronger evidence than another
            # fresh TF / native success for the same map. Preserve across restart.
            rejected=bool(self.rejected_map_name and map_name==self.rejected_map_name)
            if rejected:
                self.required=True
                self.reason='现场已确认此地图定位朝向错误；请备份后重新建图，不能仅靠定位成功反馈恢复导航'
            return {'required':self.required,'reason':self.reason,'invalid_since':self.invalid_since,'rejected_map':rejected,
                    'verified_unix':self.verified_event}
