"""Pure initial-heading policy; use the path tangent, not a line through obstacles."""
import math

def initial_heading(points,x,y,yaw):
    for point in points:
        dx,dy=point['x']-x,point['y']-y
        if math.hypot(dx,dy)>=.35:
            return math.atan2(dy,dx)
    return yaw

def heading_error(target,current):
    return math.atan2(math.sin(target-current),math.cos(target-current))

def turn_velocity(error):
    if abs(error)<=math.radians(12):return 0.
    return math.copysign(min(.5,max(.28,abs(error)*.6)),error)
