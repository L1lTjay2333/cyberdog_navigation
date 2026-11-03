"""Fixed world-space target box, initialized once from centre depth. No control."""
import numpy as np
from reconstruction_core import checked_pose, decode_depth_frame
from reconstruction_postprocess import read_depth, masked_record


class TargetBox:
    def __init__(self, size):
        values=np.asarray(size,dtype=float)
        if values.shape!=(3,) or not np.isfinite(values).all() or np.any(values<.1) or np.any(values>3):
            raise ValueError('目标范围三个尺寸须为0.1—3米')
        self.size=values;self.center=None

    def apply(self,frame):
        decode_depth_frame(frame,stride=16)  # Validate bounded payload before reading it.
        image,z=read_depth(frame)
        t,q=checked_pose(frame)
        x,y,v,w=q
        r=np.array([[1-2*(y*y+v*v),2*(x*y-v*w),2*(x*v+y*w)],
                    [2*(x*y+v*w),1-2*(x*x+v*v),2*(y*v-x*w)],
                    [2*(x*v-y*w),2*(y*v+x*w),1-2*(x*x+y*y)]])
        h,width=z.shape;fx,fy,cx,cy=frame['intrinsics']
        if self.center is None:
            # Require actual centre measurements; do not guess a missing target.
            patch=z[max(0,h//2-5):h//2+6,max(0,width//2-5):width//2+6]
            good=patch[np.isfinite(patch)&(patch>=.15)&(patch<=4)]
            if len(good)<30:raise ValueError('画面中心缺少有效深度，请将目标实体对准中央后重新开始')
            d=float(np.median(good));p=np.array([(width//2-cx)*d/fx,(h//2-cy)*d/fy,d])
            self.center=r@p+t
        valid=np.isfinite(z)&(z>=.15)&(z<=4)
        yy,xx=np.nonzero(valid);d=z[valid]
        world=np.column_stack(((xx-cx)*d/fx,(yy-cy)*d/fy,d))@r.T+t
        inside=np.all(np.abs(world-self.center)<=self.size/2,axis=1)
        mask=np.zeros(z.shape,dtype=bool);mask[yy[inside],xx[inside]]=True
        result=masked_record(frame,image,mask)
        result['target_box']={'center':self.center.tolist(),'size':self.size.tolist(),'frame':frame['frame'],
                              'selection':'first_frame_center_depth_fixed_world_box'}
        return result
