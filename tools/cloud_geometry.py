"""ROS-independent PointCloud2 decoding and rigid transforms; Python 3.6."""
import math
import struct
try:
    import numpy as _np
except ImportError:
    _np=None # Pure Python remains available for offline/embedded tooling.

def decode_xyz(msg,max_points=1200):
    if max_points<1:raise ValueError('Invalid sample limit')
    width,height,step,row=int(msg.width),int(msg.height),int(msg.point_step),int(msg.row_step)
    if min(width,height,row)<0 or step<=0 or row<width*step or len(msg.data)!=row*height:
        raise ValueError('Invalid point cloud layout')
    fields={}
    for field in msg.fields:
        if field.name in fields:raise ValueError('Duplicate cloud field')
        fields[field.name]=field
    readers=[]
    for name in ('x','y','z'):
        field=fields.get(name)
        if field is None or field.count!=1 or field.datatype not in (7,8):
            raise ValueError('Scalar floating XYZ required')
        reader=struct.Struct(('>' if msg.is_bigendian else '<')+('f' if field.datatype==7 else 'd'))
        if field.offset<0 or field.offset+reader.size>step:raise ValueError('Field outside point')
        readers.append((field.offset,reader))
    total=width*height;count=min(total,max_points);data=memoryview(msg.data);points=[]
    if _np is not None and count>=1000:
        # Exact same sample indices/field offsets, including padded rows. Avoid
        # tens of thousands of Python unpack/finite calls on the ROS computer.
        indices=_np.arange(count,dtype=_np.int64)*total//count
        rows,cols=indices//width,indices%width
        columns=[]
        for name in ('x','y','z'):
            field=fields[name]
            dtype=('>' if msg.is_bigendian else '<')+('f4' if field.datatype==7 else 'f8')
            view=_np.ndarray((height,width),dtype=dtype,buffer=data,offset=field.offset,strides=(row,step))
            columns.append(view[rows,cols])
        values=_np.column_stack(columns)
        return values[_np.isfinite(values).all(axis=1)].tolist()
    for index in range(count):
        i=index*total//count;base=(i//width)*row+(i%width)*step
        p=[reader.unpack_from(data,base+offset)[0] for offset,reader in readers]
        if all(math.isfinite(v) for v in p):points.append(p)
    return points

def transform_xyz(points,translation,quaternion):
    if len(translation)!=3 or len(quaternion)!=4 or not all(math.isfinite(v) for v in list(translation)+list(quaternion)):
        raise ValueError('Invalid rigid transform')
    translation=[float(v) for v in translation]
    quaternion=[float(v) for v in quaternion]
    norm=math.sqrt(sum(v*v for v in quaternion))
    if norm<1e-9:raise ValueError('Zero rotation quaternion')
    x,y,z,w=[v/norm for v in quaternion]
    matrix=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    # Direct multiply avoids per-point generators and NumPy-scalar dispatch.
    a,b,c=matrix;tx,ty,tz=translation;result=[]
    if _np is not None and len(points)>=1000:
        values=_np.asarray(points,dtype=float);px,py,pz=values[:,0],values[:,1],values[:,2]
        return _np.column_stack((a[0]*px+a[1]*py+a[2]*pz+tx,
            b[0]*px+b[1]*py+b[2]*pz+ty,c[0]*px+c[1]*py+c[2]*pz+tz)).tolist()
    for p in points:
        px,py,pz=float(p[0]),float(p[1]),float(p[2])
        result.append([a[0]*px+a[1]*py+a[2]*pz+tx,
                       b[0]*px+b[1]*py+b[2]*pz+ty,
                       c[0]*px+c[1]*py+c[2]*pz+tz])
    return result

class FootprintObservation:
    """Unknown cells physically occupied by the actual robot body, no padding.

    Never erase occupied lidar cells. A rotation records its witnessed swept
    footprint; unobserved space beyond that footprint remains unknown.
    """
    def __init__(self):self.poses=[]
    def observe(self,pose):
        if pose.get('frame')!='laser_odom':return
        if self.poses:
            last=self.poses[-1];d=math.hypot(pose['x']-last['x'],pose['y']-last['y'])
            turn=abs(math.atan2(math.sin(pose['yaw']-last['yaw']),math.cos(pose['yaw']-last['yaw'])))
            if d> .5:self.poses=[]
            elif d<.02 and turn<.04:return
        self.poses.append(dict(pose));self.poses=self.poses[-500:]
    def apply(self,grid):
        if grid['frame']!='laser_odom' or not self.poses:return grid
        data=list(grid['data']);w,h,r=grid['width'],grid['height'],grid['resolution'];o=grid['origin']
        angle=2*math.atan2(o['qz'],o['qw']);c,s=math.cos(angle),math.sin(angle);cleared=0
        for p in self.poses:
            dx,dy=p['x']-o['x'],p['y']-o['y'];cx,cy=math.floor((c*dx+s*dy)/r),math.floor((-s*dx+c*dy)/r)
            n=math.ceil(.29/r);pc,ps=math.cos(p['yaw']),math.sin(p['yaw'])
            for y in range(max(0,cy-n),min(h,cy+n+1)):
                for x in range(max(0,cx-n),min(w,cx+n+1)):
                    index=y*w+x
                    if data[index]!=-1:continue
                    wx,wy=o['x']+c*(x+.5)*r-s*(y+.5)*r,o['y']+s*(x+.5)*r+c*(y+.5)*r
                    bx,by=pc*(wx-p['x'])+ps*(wy-p['y']),-ps*(wx-p['x'])+pc*(wy-p['y'])
                    # Entire cell, not only its centre, must fit in the body.
                    margin=r*math.sqrt(2)/2
                    if -.23+margin<=bx<=.25-margin and abs(by)<=.13-margin:
                        data[index]=0;cleared+=1
        return dict(grid,data=data,body_observed_free_cells=cleared)
