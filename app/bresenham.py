"""Bresenham's line algorithm, implemented from first principles.

This is the ray-casting primitive `occupancy_grid.py` uses to mark every
grid cell between the sensor and a returned distance reading as "probably
free," and the cell at the reading itself as "probably occupied." No
existing SLAM/robotics/line-drawing library is used here -- this is a
hand-rolled implementation that the rest of the mapper is built on.
"""
from __future__ import annotations

from typing import List, Tuple


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Returns the list of integer grid cells on the line from (x0, y0) to
    (x1, y1) inclusive of both endpoints, using the standard integer-only
    Bresenham algorithm (no floating point, no division) generalized to
    all eight octants via the dx/dy sign + error-term formulation.
    """
    cells: List[Tuple[int, int]] = []

    dx = x1 - x0
    dy = y1 - y0
    x_step = 1 if dx >= 0 else -1
    y_step = 1 if dy >= 0 else -1
    dx = abs(dx)
    dy = abs(dy)

    x, y = x0, y0
    cells.append((x, y))

    if dx >= dy:
        # x is the driving axis: one cell per x step, y accumulates error.
        err = 2 * dy - dx
        for _ in range(dx):
            x += x_step
            if err > 0:
                y += y_step
                err += 2 * (dy - dx)
            else:
                err += 2 * dy
            cells.append((x, y))
    else:
        # y is the driving axis: mirror image of the above.
        err = 2 * dx - dy
        for _ in range(dy):
            y += y_step
            if err > 0:
                x += x_step
                err += 2 * (dx - dy)
            else:
                err += 2 * dx
            cells.append((x, y))

    return cells
