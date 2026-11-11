"""Conservative spatial cleanup of one metric depth frame before projection."""
import numpy as np


def sample_clean_depth(image, stride, scale):
    height, width = image.shape
    depth = np.pad(image.astype(float)*scale, 1, constant_values=np.nan)
    center = depth[1:height+1:stride, 1:width+1:stride]
    valid = np.isfinite(center) & (center >= .15) & (center <= 4.)
    tolerance = .012 + .004*center
    supported = np.zeros(center.shape, dtype=bool)
    smooth = valid.copy()
    total = center.copy()
    with np.errstate(invalid='ignore'):
        for row in range(3):
            for col in range(3):
                if (row, col) == (1, 1):
                    continue
                other = depth[row:row+height:stride, col:col+width:stride]
                close = (other >= .15) & (other <= 4.) & (abs(other-center) <= tolerance)
                supported |= close
                # Smooth only interiors with agreement on all four sides.
                # Depth edges and thin structures keep their measured values.
                if row == 1 or col == 1:
                    smooth &= close
                    total += np.where(close, other, 0.)
    # No extrapolation into holes or replacement of unsupported measurements.
    return np.where(valid & supported, np.where(smooth, total/5, center), np.nan)
