"""Tracks connected WebSocket clients and broadcasts JSON messages to all
of them. Kept separate from state.py so that module has no FastAPI/
Starlette dependency."""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Set

from fastapi import WebSocket

logger = logging.getLogger("lidar-slam-dashboard.ws")


class WebSocketManager:
    def __init__(self) -> None:
        self._connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._connections.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._connections.discard(ws)

    async def broadcast(self, message: dict[str, Any]) -> None:
        """Sends `message` (JSON-encoded once, reused for every client) to
        every connected client. Clients that error on send (closed/dead
        socket) are dropped rather than left to poison future broadcasts."""
        if not self._connections:
            return
        payload = json.dumps(message)
        dead: Set[WebSocket] = set()
        async with self._lock:
            targets = list(self._connections)
        for ws in targets:
            try:
                await ws.send_text(payload)
            except Exception as exc:  # noqa: BLE001 -- any send failure means a dead socket
                logger.warning("dropping dead websocket client: %s", exc)
                dead.add(ws)
        if dead:
            async with self._lock:
                self._connections -= dead
