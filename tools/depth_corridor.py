"""Check observed rays, not empty obstacle ROI, for a flat indoor corridor."""
import math

def observed_corridor(points,origin,plane=1.):
    """Require all 3x3 lateral/height bins to contain measured long depth rays.

    The far endpoints may be floor/ceiling/outside the obstacle height ROI.
    A missing measurement contributes nothing; this never turns empty data free.
    Returns an observed forward bound, NOT an unlimited clear-space distance.
    """
    bins=set();counts={};ox,oy,oz=origin
    for x,y,z in points:
        if not all(math.isfinite(v) for v in (x,y,z)) or x<plane or x<=ox:continue
        fraction=(plane-ox)/(x-ox)
        if not 0<fraction<=1:continue
        cy=oy+fraction*(y-oy);cz=oz+fraction*(z-oz)
        if not -.35<=cy<=.35 or not .03<=cz<=.35:continue
        key=(min(2,int((cy+.35)/(.7/3))),min(2,int((cz-.03)/(.32/3))))
        counts[key]=counts.get(key,0)+1
        if counts[key]>=2:bins.add(key)
    return plane if len(bins)==9 else None
