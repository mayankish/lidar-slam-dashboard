"""FastAPI application entry point.

Run with:

    python -m app.main

from the `lidar-slam-dashboard/` directory (not `uvicorn app.main:app`
directly -- see README "Build instructions" for why this project is
structured to be run as a module rather than relying on uvicorn's CLI
import-string reload machinery, which isn't needed here since this app
has no hot-reload requirement).
"""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

import socket
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import contract, control_client
from .state import AppState
from .udp_listener import start_udp_listener
from .ws_manager import WebSocketManager

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
logger = logging.getLogger("lidar-slam-dashboard")

STATIC_DIR = Path(__file__).parent / "static"

state = AppState()
ws_manager = WebSocketManager()
_udp_transport: Optional[asyncio.DatagramTransport] = None


async def _broadcast_grid_update() -> None:
    async with state.lock:
        snapshot = state.grid.to_serializable()
        sweep_dir = state.last_sweep_dir
    # last_sweep_dir travels as a sibling key, not inside `grid`, so
    # OccupancyGrid.to_serializable() stays a pure grid-only snapshot.
    await ws_manager.broadcast({"kind": "grid_update", "grid": snapshot, "last_sweep_dir": sweep_dir})


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _udp_transport
    _udp_transport = await start_udp_listener(state, on_sweep_complete=_broadcast_grid_update)
    logger.info("lidar-slam-dashboard up: UDP :5005 listener active, serving HTTP/WS")
    try:
        yield
    finally:
        if _udp_transport is not None:
            _udp_transport.close()


app = FastAPI(title="lidar-slam-dashboard", lifespan=lifespan)


class ControlRequest(BaseModel):
    cmd: str  # one of: start_scan, stop_scan, set_sweep_range, ping
    param1: int = 0
    param2: int = 0


_CMD_NAME_TO_ID = {
    "start_scan": contract.CmdId.START_SCAN,
    "stop_scan": contract.CmdId.STOP_SCAN,
    "set_sweep_range": contract.CmdId.SET_SWEEP_RANGE,
    "ping": contract.CmdId.PING,
}


@app.get("/status")
async def get_status() -> dict:
    async with state.lock:
        return state.status_dict()


@app.post("/control")
async def post_control(req: ControlRequest) -> dict:
    cmd_id = _CMD_NAME_TO_ID.get(req.cmd)
    if cmd_id is None:
        raise HTTPException(status_code=400, detail=f"unknown cmd '{req.cmd}', expected one of {list(_CMD_NAME_TO_ID)}")
    try:
        control_client.send_control_command(cmd_id, req.param1, req.param2)
    except socket.gaierror as exc:
        raise HTTPException(
            status_code=502,
            detail=f"could not resolve base-radio host: {exc}. "
                   f"Set the LIDARBASE_HOST environment variable to its IP address.",
        ) from exc
    return {"sent": req.cmd, "param1": req.param1, "param2": req.param2}


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    try:
        async with state.lock:
            snapshot = state.grid.to_serializable()
            sweep_dir = state.last_sweep_dir
        await websocket.send_json({"kind": "grid_update", "grid": snapshot, "last_sweep_dir": sweep_dir})
        while True:
            # This dashboard only ever pushes (grid updates); it has no use
            # for inbound WS messages, but the receive loop must run so the
            # server notices a client disconnect promptly instead of
            # leaking a dead connection into ws_manager's broadcast set.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await ws_manager.disconnect(websocket)


# Mounted last so it doesn't shadow the API routes above; serves
# static/index.html at "/" and static/app.js etc. alongside it.
app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
