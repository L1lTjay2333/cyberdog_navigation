"""Convert measured XYZ in laser_frame to a planar scan, leaving gaps unknown."""
import math

ANGLE_MIN=-math.pi/2
ANGLE_INCREMENT=math.pi/360
RANGE_MIN=.2
RANGE_MAX=5.5

def points_to_scan(points):
    ranges=[float('nan')]*361
    for x,y,z in points:
        if not all(math.isfinite(v) for v in (x,y,z)) or not -.1<=z<=.5 or x<=0:continue
        distance=math.hypot(x,y)
        if not RANGE_MIN<=distance<=RANGE_MAX:continue
        angle=math.atan2(y,x)
        index=int(round((angle-ANGLE_MIN)/ANGLE_INCREMENT))
        if 0<=index<len(ranges) and (not math.isfinite(ranges[index]) or distance<ranges[index]):
            ranges[index]=distance
    return ranges
