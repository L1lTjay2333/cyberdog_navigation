"""Conservative native optical-camera cloud check, independent of ROS."""
import math
from cloud_geometry import decode_xyz,transform_xyz
from depth_corridor import observed_corridor

def forward_cloud_distance(msg,translation,quaternion):
    count=min(int(msg.width)*int(msg.height),20000)
    points=[p for p in decode_xyz(msg,20000) if p[2]>.01]
    fraction=len(points)/float(count) if count else 0
    points=transform_xyz(points,translation,quaternion)
    return forward_points_distance(points,count,fraction,translation)

def forward_points_distance(points,count,fraction,origin):
    """Same measured-corridor rule for images and native clouds.

    Peripheral invalid pixels must not veto a completely observed forward
    corridor. Keep isolated close returns, the same ROI and freshness limits.
    """
    if count<1000:
        raise ValueError('深度采样覆盖不足')
    # A measured far wall is still a real distance, not "missing depth".
    xs=[p[0] for p in points if p[0]>=.1 and abs(p[1])<=.35 and .03<=p[2]<=1]
    # Even sparse near measurements must stop the robot, never be discarded.
    if xs and min(xs)<=.45:return min(xs),len(xs)
    if fraction<.8 or len(xs)<10:
        # Missing peripheral pixels do not prove the forward corridor invalid.
        # Require actual rays in all nine forward bins, not a reduced threshold.
        bound=observed_corridor(points,origin)
        if bound is None:
            raise ValueError('前方测量射线覆盖不足（整帧有效率 %.0f%%）'%(fraction*100))
        return min(xs+[bound]),len(xs)
    return min(xs),len(xs)
