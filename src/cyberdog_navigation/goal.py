"""Validate a proposed map goal without sending any robot command."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class MapGoal:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'

    def validate(self, occupied, resolution, origin=(0.0, 0.0)):
        if self.frame_id != 'map':
            raise ValueError('Goal must use map coordinates')
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw, resolution, *origin)) or resolution <= 0:
            raise ValueError('Invalid coordinates or map resolution')
        if not occupied or not occupied[0] or any(len(row) != len(occupied[0]) for row in occupied):
            raise ValueError('Invalid occupancy grid')
        ix = math.floor((self.x-origin[0])/resolution)
        iy = math.floor((self.y-origin[1])/resolution)
        if not (0 <= iy < len(occupied) and 0 <= ix < len(occupied[0])):
            raise ValueError('Goal outside map')
        if occupied[iy][ix] != 0:
            raise ValueError('Goal is occupied or unknown')
        return ix, iy
