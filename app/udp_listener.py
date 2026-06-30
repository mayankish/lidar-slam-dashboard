"""asyncio UDP listener for telemetry broadcast by base-radio on :5005.

Reconstructs sweeps: accumulates scan_sample frames into the current
sweep's point list, and on scan_complete hands the whole sweep to the
OccupancyGrid for ray-casting, then clears it to start fresh -- mirroring
the same "accumulate between scan_complete markers" design used in
lidar-android-app's LidarViewModel.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Optional

from . import contract
from .state import AppState, HealthSnapshot

logger = logging.getLogger("lidar-slam-dashboard.udp")

TELEMETRY_PORT = 5005

# Called after each completed sweep is integrated into the grid, and after
# each health_status update, so main.py can push a WS broadcast without
# this module importing the websocket manager directly.
OnSweepComplete = Callable[[], Awaitable[None]]


class TelemetryProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: AppState, on_sweep_complete: Optional[OnSweepComplete] = None):
        self.state = state
        self.on_sweep_complete = on_sweep_complete
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]
        logger.info("UDP telemetry listener bound on :%d", TELEMETRY_PORT)

    def datagram_received(self, data: bytes, addr) -> None:
        # datagram_received is synchronous (asyncio's transport API), but
        # frame handling needs to await the state lock -- schedule it as a
        # task rather than blocking the event loop's UDP callback.
        asyncio.ensure_future(self._handle_datagram(data))

    def error_received(self, exc: Exception) -> None:
        logger.warning("UDP transport error: %s", exc)

    async def _handle_datagram(self, data: bytes) -> None:
        if len(data) != contract.WIRE_LEN:
            return  # base-radio only ever sends WIRE_LEN datagrams; ignore stray traffic on this port
        frame = contract.unpack(data)
        async with self.state.lock:
            self.state.link_stats.frames_received += 1
            self.state.link_stats.last_frame_at = _now()
            if frame is None:
                self.state.link_stats.frames_crc_failed += 1
                return
            self.state.link_stats.track_sequence(frame.seq)

            if frame.type == contract.Type.SCAN_SAMPLE:
                sample = contract.decode_scan_sample(frame)
                if sample.distance_mm != contract.OUT_OF_RANGE:
                    self.state.current_sweep.append((sample.angle_cdeg, sample.distance_mm))
                else:
                    # Still ray-cast out-of-range readings so the grid's
                    # free-space clearing reflects "nothing detected" rays
                    # too -- see OccupancyGrid.integrate_point.
                    self.state.current_sweep.append((sample.angle_cdeg, contract.OUT_OF_RANGE))
            elif frame.type == contract.Type.SCAN_COMPLETE:
                sweep = self.state.current_sweep
                self.state.current_sweep = []
                self.state.last_sweep_dir = contract.decode_scan_complete(frame).sweep_dir
                if sweep:
                    self.state.grid.integrate_sweep(sweep)
                    self.state.link_stats.sweeps_completed += 1
            elif frame.type == contract.Type.HEALTH_STATUS:
                h = contract.decode_health_status(frame)
                self.state.health = HealthSnapshot(
                    fault_flags=h.fault_flags,
                    battery_mv=h.battery_mv,
                    timestamp_ms=h.timestamp_ms,
                )
            elif frame.type == contract.Type.CONTROL_ACK:
                ack = contract.decode_control_ack(frame)
                logger.info("control_ack: cmd_id=0x%02x status=%d", ack.cmd_id, ack.status)
            # else: unknown type, dropped silently (matches every other
            # layer's handling of a data-contract violation upstream).

        if frame is not None and frame.type == contract.Type.SCAN_COMPLETE and self.on_sweep_complete:
            await self.on_sweep_complete()


def _now() -> float:
    import time
    return time.time()


async def start_udp_listener(
    state: AppState, on_sweep_complete: Optional[OnSweepComplete] = None
) -> asyncio.DatagramTransport:
    """Binds the UDP telemetry socket with SO_REUSEADDR (so a dashboard
    restart doesn't have to wait out the OS's TIME_WAIT/lingering-socket
    window on the same port) and returns the transport so the caller
    (main.py's lifespan handler) can close it on shutdown."""
    loop = asyncio.get_running_loop()
    # asyncio's create_datagram_endpoint() sets SO_REUSEADDR on the
    # underlying socket by default on POSIX platforms for UDP sockets, so
    # a dashboard restart can immediately rebind :5005 rather than waiting
    # out a lingering-socket window -- no explicit reuse_address= kwarg is
    # needed (and that kwarg has been removed in newer Python versions).
    transport, _protocol = await loop.create_datagram_endpoint(
        lambda: TelemetryProtocol(state, on_sweep_complete),
        local_addr=("0.0.0.0", TELEMETRY_PORT),
        allow_broadcast=True,
    )
    return transport
