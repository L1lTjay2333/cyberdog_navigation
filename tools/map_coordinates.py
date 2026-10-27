"""Map origin rotation and ROS-grid-to-canvas conversion, without ROS imports."""
import math

def project_pose(grid, pose, canvas_width=900, canvas_height=600):
    if pose['frame'] != grid['frame']:
        raise ValueError('Pose and map frames do not match')
    width, height, resolution = grid['width'], grid['height'], grid['resolution']
    origin = grid['origin']
    values = (pose['x'], pose['y'], pose['yaw'], origin['x'], origin['y'], origin['qz'], origin['qw'], resolution)
    if not all(math.isfinite(v) for v in values) or min(width,height,resolution,canvas_width,canvas_height) <= 0:
        raise ValueError('Invalid map or pose')
    if origin['qz']**2+origin['qw']**2 < 1e-12:
        raise ValueError('Invalid origin quaternion')
    angle = 2*math.atan2(origin['qz'],origin['qw'])
    dx,dy = pose['x']-origin['x'], pose['y']-origin['y']
    gx = (math.cos(angle)*dx+math.sin(angle)*dy)/resolution
    gy = (-math.sin(angle)*dx+math.cos(angle)*dy)/resolution
    if not (0 <= gx < width and 0 <= gy < height):
        raise ValueError('Pose outside map')
    scale = min(canvas_width/width,canvas_height/height)
    return {'x':gx*scale, 'y':(height-gy)*scale, 'angle':-(pose['yaw']-angle)}
