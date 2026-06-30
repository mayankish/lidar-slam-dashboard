"""Hand-rolled occupancy-grid mapper, built on `bresenham.py`'s ray-caster.

v1 scope (see README "Known limitations" and the stretch-goal note in
../README.md): this assumes a **stationary scanning head** at a fixed,
known position and orientation in the grid -- there is no odometry, no
pose estimation, and no loop closure. Each completed sweep is ray-cast
directly into the grid from the same fixed origin cell. This is
deliberately *not* SLAM (Simultaneous Localization And Mapping) in the
formal sense -- there is no localization step, because the platform
this v1 targets never moves. Moving-rover SLAM (odometry fusion, pose
graph, scan matching) is documented as a stretch goal only; see
extras/ in the repo root for where that would live if built.

Model: a simple two-counter (hit/miss) log-odds-free occupancy grid.
Every ray increments a "free" counter for every cell it passes through
and a "hit" counter for the cell at its endpoint (when the reading is a
valid, in-range distance). The exported probability for a cell is
hits / (hits + misses), which converges to a stable estimate over many
sweeps and degrades gracefully for cells seen only once or twice --
intentionally simpler than a Bayesian log-odds update (e.g. the
classic occupancy-grid-mapping algorithm from Moravec & Elfes), which
would be the natural place to extend this for noisier sensors.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .bresenham import bresenham_line

OUT_OF_RANGE = 0xFFFF


@dataclass
class GridConfig:
    width_m: float = 6.0
    height_m: float = 6.0
    resolution_m: float = 0.02  # 2 cm/cell
    # Sensor origin, in grid cells, where angle_cdeg=0 points along +x.
    # Defaults to the horizontal center, near the bottom edge, matching a
    # sensor that sweeps roughly "outward and upward" across the grid in
    # front of it (sweep range is configurable at the firmware/UI level;
    # the grid doesn't need to know the configured min/max angle).
    origin_frac_x: float = 0.5
    origin_frac_y: float = 0.95

    @property
    def width_cells(self) -> int:
        return max(1, int(round(self.width_m / self.resolution_m)))

    @property
    def height_cells(self) -> int:
        return max(1, int(round(self.height_m / self.resolution_m)))


class OccupancyGrid:
    def __init__(self, config: Optional[GridConfig] = None):
        self.config = config or GridConfig()
        w, h = self.config.width_cells, self.config.height_cells
        self.hits = np.zeros((h, w), dtype=np.uint16)
        self.misses = np.zeros((h, w), dtype=np.uint16)
        self.origin_x = int(round(self.config.origin_frac_x * w))
        self.origin_y = int(round(self.config.origin_frac_y * h))
        self.sweep_count = 0

    def _in_bounds(self, x: int, y: int) -> bool:
        h, w = self.hits.shape
        return 0 <= x < w and 0 <= y < h

    def integrate_point(self, angle_cdeg: int, distance_mm: int) -> None:
        """Ray-casts a single scan_sample into the grid. Out-of-range
        readings (distance_mm == OUT_OF_RANGE) still clear the ray up to
        the grid boundary as free space (the sensor saw nothing within
        range, which is itself information) but mark no occupied cell."""
        angle_rad = math.radians(angle_cdeg / 100.0)
        # Screen/grid convention: +x to the right, +y "up" the grid (away
        # from the sensor), matching the Android app's polar-to-cartesian
        # convention in ui/LidarScreen.kt for visual consistency across
        # clients, modulo the obvious axis-orientation difference between
        # a phone Canvas (+y down) and this grid (+y up, row 0 = far edge,
        # see to_serializable()'s row-order note).
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)

        max_range_cells = max(self.hits.shape) * 2  # generous; clipped below
        if distance_mm == OUT_OF_RANGE:
            ray_cells = max_range_cells
            hit_valid = False
        else:
            ray_cells = int(round((distance_mm / 1000.0) / self.config.resolution_m))
            hit_valid = True

        end_x = self.origin_x + int(round(dx * ray_cells))
        end_y = self.origin_y - int(round(dy * ray_cells))  # row 0 = far edge, see to_serializable

        cells = bresenham_line(self.origin_x, self.origin_y, end_x, end_y)

        # Walk the ray; everything before the final in-bounds cell is
        # "free," the final cell is "occupied" only for a valid in-range
        # reading. If the ray leaves the grid before reaching its nominal
        # endpoint, we simply stop -- cells outside the grid don't exist.
        last_in_bounds_idx = -1
        for i, (cx, cy) in enumerate(cells):
            if self._in_bounds(cx, cy):
                last_in_bounds_idx = i

        for i in range(last_in_bounds_idx + 1):
            cx, cy = cells[i]
            if not self._in_bounds(cx, cy):
                continue
            if i == last_in_bounds_idx and hit_valid and i == len(cells) - 1:
                self.hits[cy, cx] += 1
            else:
                self.misses[cy, cx] += 1

    def integrate_sweep(self, samples: List[tuple]) -> None:
        """`samples` is a list of (angle_cdeg, distance_mm) tuples, the
        accumulated scan_sample readings for one completed sweep (i.e.
        everything received between two scan_complete markers -- see
        udp_listener.py's SweepAccumulator)."""
        for angle_cdeg, distance_mm in samples:
            self.integrate_point(angle_cdeg, distance_mm)
        self.sweep_count += 1

    def probability_grid(self) -> np.ndarray:
        """Returns a float32 array, same shape as the grid, of P(occupied)
        per cell in [0, 1]. Cells with no observations at all (hits ==
        misses == 0) are returned as -1.0 to distinguish "unknown" from
        "observed and found free" (0.0) for the frontend to render
        differently."""
        total = self.hits.astype(np.float32) + self.misses.astype(np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            prob = np.where(total > 0, self.hits.astype(np.float32) / total, -1.0)
        return prob

    def to_serializable(self) -> dict:
        """JSON-friendly snapshot for the /status endpoint and WS pushes.
        Probabilities are quantized to integers in [-1, 100] (-1 =
        unknown) to keep the payload compact over the websocket; the
        static frontend (`static/app.js`) un-quantizes by dividing by 100.
        Row 0 of `cells` is the far edge of the grid (max +y, i.e.
        farthest from the sensor origin) and the last row is nearest the
        sensor -- matching how `integrate_point` walks +y "up" the grid
        away from the origin.
        """
        prob = self.probability_grid()
        quantized = np.where(prob < 0, -1, np.round(prob * 100)).astype(np.int16)
        return {
            "width_cells": self.config.width_cells,
            "height_cells": self.config.height_cells,
            "resolution_m": self.config.resolution_m,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "sweep_count": self.sweep_count,
            "cells": quantized.tolist(),
        }
