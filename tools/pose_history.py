"""Time-aligned native dog_pose history. Does not broadcast or invent ROS TF."""
from collections import deque
import math

class PoseHistory:
    def __init__(self,frame='laser_odom',capacity=100):self.samples=deque(maxlen=capacity);self.frame=frame
    def add(self,stamp,position,quaternion,frame):
        if frame!=self.frame or stamp<=0 or not all(math.isfinite(v) for v in [stamp]+list(position)+list(quaternion)):return
        norm=math.sqrt(sum(v*v for v in quaternion))
        if norm<1e-6:return
        q=[v/norm for v in quaternion]
        if self.samples and stamp<=self.samples[-1][0]:return
        self.samples.append((stamp,list(position),q))
    def latest(self,now):
        if not self.samples or not -.05<=now-self.samples[-1][0]<=.3:raise ValueError('原厂建图位姿过期')
        return self.make(*self.samples[-1])
    @staticmethod
    def make(stamp,p,q):
        return {'x':p[0],'y':p[1],'z':p[2],'quaternion':q,
            'yaw':math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])),
            'frame':'laser_odom','received_unix':stamp,'source':'stock_dog_pose'}
    def at(self,stamp):
        samples=list(self.samples)
        if not samples:raise ValueError('位姿历史为空')
        for a,b in zip(samples,samples[1:]):
            if a[0]<=stamp<=b[0]:
                if b[0]-a[0]>.15:raise ValueError('位姿采样间隔过大')
                t=(stamp-a[0])/(b[0]-a[0]);p=[x+(y-x)*t for x,y in zip(a[1],b[1])]
                sign=1 if sum(x*y for x,y in zip(a[2],b[2]))>=0 else -1
                q=[x+(sign*y-x)*t for x,y in zip(a[2],b[2])];norm=math.sqrt(sum(x*x for x in q))
                return self.make(stamp,p,[x/norm for x in q])
        nearest=min(samples,key=lambda a:abs(a[0]-stamp))
        if abs(nearest[0]-stamp)>.03:raise ValueError('深度时间戳无对应位姿')
        return self.make(*nearest)

def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]

class CompensatedPose:
    """Planar SLAM correction plus measured odom increments, never a TF broadcast.

    Installed map_builder consumes this same odom_out. Only x/y/yaw increments
    are used, not its ground-relative height or an invented 3D body extrinsic.
    """
    def __init__(self,slam,odom):self.slam=slam;self.odom=odom
    def at(self,stamp,now):
        if not self.slam.samples or not self.odom.samples:raise ValueError('等待建图及里程计历史')
        anchor=self.slam.make(*self.slam.samples[-1])
        age=now-anchor['received_unix']
        if not -.05<=age<=1.5:raise ValueError('建图校正超过补偿期限')
        if not -.05<=now-self.odom.samples[-1][0]<=.15:raise ValueError('实时里程计过期')
        if not -.05<=now-stamp<=.5:raise ValueError('请求补偿时间过期')
        start=self.odom.at(anchor['received_unix']);end=self.odom.at(stamp)
        dx,dy=end['x']-start['x'],end['y']-start['y']
        turn=math.atan2(math.sin(end['yaw']-start['yaw']),math.cos(end['yaw']-start['yaw']))
        if math.hypot(dx,dy)>.35 or abs(turn)>.5:raise ValueError('补偿期间里程计突变')
        heading=anchor['yaw']-start['yaw'];c,s=math.cos(heading),math.sin(heading)
        q=multiply([0,0,math.sin(turn/2),math.cos(turn/2)],anchor['quaternion'])
        pose=self.slam.make(stamp,[anchor['x']+c*dx-s*dy,anchor['y']+s*dx+c*dy,anchor['z']],q)
        pose.update(source='stock_slam_plus_measured_planar_odom',slam_age_s=age,compensation_m=math.hypot(dx,dy))
        return pose
    def latest(self,now):
        if not self.odom.samples:raise ValueError('里程计历史为空')
        return self.at(self.odom.samples[-1][0],now)

class ScanOdometry:
    """Bounded in-place scan only; measured odom relative to one valid map anchor.

    Does not replace localization for translation/navigation or broadcast TF.
    """
    def __init__(self,anchor,odom):
        if anchor.get('frame')!='laser_odom':raise ValueError('补扫起点坐标系无效')
        self.anchor=dict(anchor);self.odom=odom
        self.start=odom.at(anchor['received_unix'])
        self.previous=self.start
    def latest(self,now):
        if not self.odom.samples:raise ValueError('里程计历史为空')
        stamp=self.odom.samples[-1][0]
        if not -.05<=now-stamp<=.15:raise ValueError('实时补扫里程计过期')
        if now-self.anchor['received_unix']>42:raise ValueError('补扫起点期限已到')
        end=self.odom.at(stamp)
        delta=math.atan2(math.sin(end['yaw']-self.previous['yaw']),math.cos(end['yaw']-self.previous['yaw']))
        if math.hypot(end['x']-self.previous['x'],end['y']-self.previous['y'])>.08 or abs(delta)>.15:
            raise ValueError('补扫里程计突变')
        dx,dy=end['x']-self.start['x'],end['y']-self.start['y']
        if math.hypot(dx,dy)>.15:raise ValueError('原地补扫出现平移，结束')
        turn=math.atan2(math.sin(end['yaw']-self.start['yaw']),math.cos(end['yaw']-self.start['yaw']))
        heading=self.anchor['yaw']-self.start['yaw'];c,s=math.cos(heading),math.sin(heading)
        q=multiply([0,0,math.sin(turn/2),math.cos(turn/2)],self.anchor['quaternion'])
        pose=PoseHistory.make(stamp,[self.anchor['x']+c*dx-s*dy,self.anchor['y']+s*dx+c*dy,self.anchor['z']],q)
        pose.update(source='scan_anchor_plus_measured_odom',anchor_unix=self.anchor['received_unix'])
        self.previous=end
        return pose
