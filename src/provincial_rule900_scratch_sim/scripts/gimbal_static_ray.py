"""Equivalent static ray occupancy query; retains every original sample cell."""
import math
from functools import lru_cache

import numpy as np


@lru_cache(maxsize=512)
def _indices(count):
    return np.arange(1, count+1, dtype=float)


def ray_has_unmapped_return(grid, origin, angle, distance, margin=.60):
    if not all(math.isfinite(v) for v in (*origin, angle, distance, margin, grid.resolution)):
        raise ValueError('nonfinite ray query')
    step = grid.resolution*.5
    limit = distance+margin
    if step <= 0 or limit < 0:
        raise ValueError('invalid ray sampling extent')
    count = int(math.ceil(limit/step))
    radius = np.minimum(_indices(count)*step, limit)
    # Same operations, origin, half-cell step, endpoint and threshold as GridRoute.
    x = origin[0]+radius*math.cos(angle)
    y = origin[1]+radius*math.sin(angle)
    cols = np.floor((x-grid.origin_x)/grid.resolution).astype(np.int64)
    rows = grid.height-1-np.floor((y-grid.origin_y)/grid.resolution).astype(np.int64)
    if not np.all((rows>=0)&(rows<grid.height)&(cols>=0)&(cols<grid.width)):
        return False
    return bool(np.all(grid.pixels[rows,cols]>=250))
