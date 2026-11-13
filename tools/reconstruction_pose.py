"""Read-only camera poses from native mapping or visual odometry (robot Python 3.6)."""
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer
from rclpy.qos import qos_profile_sensor_data

class ReconstructionPose:
    def __init__(self,node,tf_buffer,namespace,frame=None):
        self.buffer=tf_buffer
        self.mapping=Buffer()
        self.frame=frame
        self.subscription=node.create_subscription(PoseStamped,namespace+'/dog_pose',self.receive,qos_profile_sensor_data)

    def receive(self,msg):
        p,q=msg.pose.position,msg.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        norm=math.sqrt(sum(v*v for v in values[3:]))
        if (msg.header.frame_id!='laser_odom' or not all(math.isfinite(v) for v in values)
                or norm<1e-6 or msg.header.stamp.sec+msg.header.stamp.nanosec/1e9<=0):return
        t=TransformStamped();t.header=msg.header;t.child_frame_id='base_link'
        t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
        t.transform.rotation.x=q.x/norm;t.transform.rotation.y=q.y/norm
        t.transform.rotation.z=q.z/norm;t.transform.rotation.w=q.w/norm
        self.mapping.set_transform(t,'stock_dog_pose')

    def lookup(self,source,stamp):
        error=None
        for frame in ([self.frame] if self.frame else ['laser_odom','vodom']):
            try:
                if frame=='laser_odom':
                    base=self.mapping.lookup_transform(frame,'base_link',stamp)
                    local=self.buffer.lookup_transform('base_link',source,stamp)
                    a,b=base.transform.rotation,local.transform.rotation
                    p=local.transform.translation
                    # Rotate the measured camera extrinsic by the measured body pose.
                    ux,uy,uz=2*(a.y*p.z-a.z*p.y),2*(a.z*p.x-a.x*p.z),2*(a.x*p.y-a.y*p.x)
                    out=TransformStamped();out.header=base.header;out.child_frame_id=source
                    t=out.transform.translation;v=base.transform.translation
                    t.x=v.x+p.x+a.w*ux+a.y*uz-a.z*uy
                    t.y=v.y+p.y+a.w*uy+a.z*ux-a.x*uz
                    t.z=v.z+p.z+a.w*uz+a.x*uy-a.y*ux
                    q=out.transform.rotation
                    q.x=a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y
                    q.y=a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x
                    q.z=a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
                    q.w=a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z
                else:out=self.buffer.lookup_transform(frame,source,stamp)
                p,q=out.transform.translation,out.transform.rotation
                values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                ts=out.header.stamp.sec+out.header.stamp.nanosec/1e9
                if not all(math.isfinite(v) for v in values) or abs(ts-stamp.nanoseconds/1e9)>1e-6:
                    raise ValueError('Pose timestamp or values invalid')
                if not .5<=math.sqrt(sum(v*v for v in values[3:]))<=1.5:raise ValueError('Pose rotation invalid')
                self.frame=frame # Never switch coordinates within a capture.
                return out
            except Exception as exc:error=exc
        raise error
