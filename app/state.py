"""Shared, in-process application state.

A single instance of `AppState` is created in `main.py` and threaded
through the UDP listener, the FastAPI route handlers, and the WebSocket
broadcaster. Kept deliberately simple (plain attributes + an asyncio
lock where mutation needs to be atomic across awaits) rather than a
class hierarchy, since this is a single-process, single-grid v1 app --
see README "Known limitations" for what multi-client/multi-grid support
would require.
"""
from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .occupancy_grid import OccupancyGrid


@dataclass
class HealthSnapshot:
    fault_flags: int
    battery_mv: int
    timestamp_ms: int
    received_at: float = field(default_factory=time.time)


@dataclass
class LinkStats:
    """Counters surfaced on GET /status for at-a-glance link health."""
    frames_received: int = 0
    frames_crc_failed: int = 0
    sweeps_completed: int = 0
    last_frame_at: Optional[float] = None
    seq_initialized: bool = False
    expected_seq: int = 0
    total_lost: int = 0

    def track_sequence(self, seq: int) -> None:
        """Mirrors esp32-raw-mac-radio/base-radio's track_sequence(): one
        shared running expected-seq counter across all frame types,
        since the STM32 emits all outgoing frames from one monotonic seq
        counter (see uart1_send_frame() in stm32-lidar-firmware).
        """
        if not self.seq_initialized:
            self.seq_initialized = True
            self.expected_seq = (seq + 1) & 0xFFFF
            return
        if seq != self.expected_seq:
            gap = (seq - self.expected_seq) & 0xFFFF
            self.total_lost += gap
        self.expected_seq = (seq + 1) & 0xFFFF


class AppState:
    def __init__(self) -> None:
        self.grid = OccupancyGrid()
        self.health: Optional[HealthSnapshot] = None
        self.link_stats = LinkStats()
        self.current_sweep: List[Tuple[int, int]] = []
        # sweep_dir of the most recently completed sweep (0=forward,
        # 1=reverse per data_contract's scan_complete payload), or None
        # before the first sweep finishes. Purely a display nicety for the
        # frontend's "last sweep: forward/reverse" label -- nothing in the
        # grid/mapping logic depends on it.
        self.last_sweep_dir: Optional[int] = None
        self.lock = asyncio.Lock()
        # WebSocket subscribers (see ws_manager.py) are intentionally not
        # stored here -- ws_manager owns its own connection set so this
        # module has no FastAPI/Starlette import dependency.

    def status_dict(self) -> dict:
        health = None
        if self.health is not None:
            health = {
                "fault_flags": self.health.fault_flags,
                "battery_mv": self.health.battery_mv,
                "timestamp_ms": self.health.timestamp_ms,
                "received_at": self.health.received_at,
            }
        return {
            "health": health,
            "last_sweep_dir": self.last_sweep_dir,
            "link_stats": {
                "frames_received": self.link_stats.frames_received,
                "frames_crc_failed": self.link_stats.frames_crc_failed,
                "sweeps_completed": self.link_stats.sweeps_completed,
                "last_frame_at": self.link_stats.last_frame_at,
                "total_lost": self.link_stats.total_lost,
            },
            "grid": {
                "width_cells": self.grid.config.width_cells,
                "height_cells": self.grid.config.height_cells,
                "resolution_m": self.grid.config.resolution_m,
                "sweep_count": self.grid.sweep_count,
            },
        }
