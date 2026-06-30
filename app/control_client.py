"""Sends control_command frames to base-radio over UDP unicast, port 5006.

Host resolution: defaults to the hostname `lidarbase.local`, resolved via
the standard library's `socket.gethostbyname()`. Unlike the Android app
(`lidar-android-app`, which has no OS-level mDNS support and so implements
its own RFC 6762 client in `MdnsResolver.kt`), a desktop/server OS running
this dashboard typically *does* have working `.local` resolution already
-- Avahi (most Linux desktops/distros) or Bonjour/mDNSResponder (macOS,
and Windows with Apple's Bonjour service or iTunes installed) both hook
into the system resolver, which is exactly what `gethostbyname()` uses.
Where that isn't true (minimal servers, some Linux distros without
nss-mdns installed, Windows without Bonjour), set the `LIDARBASE_HOST`
environment variable to base-radio's IP address directly -- see README
"Configuration."
"""
from __future__ import annotations

import logging
import os
import socket

from . import contract

logger = logging.getLogger("lidar-slam-dashboard.control")

CONTROL_PORT = 5006
DEFAULT_HOST = "lidarbase.local"


def _target_host() -> str:
    return os.environ.get("LIDARBASE_HOST", DEFAULT_HOST)


def send_control_command(cmd_id: int, param1: int = 0, param2: int = 0) -> None:
    """Resolves the configured base-radio host and sends one
    control_command datagram. Raises socket.gaierror if the host can't be
    resolved (e.g. mDNS unavailable and LIDARBASE_HOST unset/wrong) --
    callers (the /control route handler) are expected to catch this and
    surface it as an HTTP error rather than a crash, since a missing base
    station is an expected operating condition, not a bug.
    """
    host = _target_host()
    ip = socket.gethostbyname(host)  # raises socket.gaierror on failure
    wire = contract.encode_control_command(cmd_id, param1, param2)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(wire, (ip, CONTROL_PORT))
    logger.info("sent control_command cmd_id=0x%02x param1=%d param2=%d to %s:%d",
                cmd_id, param1, param2, ip, CONTROL_PORT)
