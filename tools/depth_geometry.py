"""Rectified depth image to sampled XYZ, no ROS dependency or control outputs."""
import math
import struct

def manual_sample_stride(width,height):
    # Fixed stride=16 produces only 300 samples for 320x240 images and always
    # fails the 1000-sample guard. Maintain ~1200 samples across resolutions.
    return max(1,int(math.sqrt(max(0,width*height)/1200.)))

def depth_xyz(image, info, stride=4):
    if stride < 1 or image.width != info.width or image.height != info.height:
        raise ValueError('Invalid sampling or mismatched calibration dimensions')
    if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
        raise ValueError('Depth and calibration frames differ')
    if image.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Metric depth encoding required')
    fmt = 'H' if image.encoding == '16UC1' else 'f'
    reader = struct.Struct(('>' if image.is_bigendian else '<') + fmt)
    scale = .001 if fmt == 'H' else 1.
    if image.step < image.width * reader.size or len(image.data) != image.step * image.height:
        raise ValueError('Invalid depth buffer layout')
    if len(info.p) != 12 or any(not math.isfinite(x) for x in info.p):
        raise ValueError('Invalid rectified projection')
    # Galactic CameraInfo arrays may contain NumPy scalars. Convert ONCE,
    # avoiding thousands of scalar NumPy operations in the Python pixel loop.
    fx,fy,cx,cy = (float(info.p[i]) for i in (0,5,2,6))
    if fx <= 0 or fy <= 0 or info.p[3] != 0 or info.p[7] != 0:
        raise ValueError('Unsupported projection')
    points=[];sampled=0
    for v in range(0,image.height,stride):
        for u in range(0,image.width,stride):
            sampled += 1
            z=reader.unpack_from(image.data,v*image.step+u*reader.size)[0]*scale
            if math.isfinite(z) and z > 0:
                points.append([(u-cx)*z/fx,(v-cy)*z/fy,z])
    return points,sampled
