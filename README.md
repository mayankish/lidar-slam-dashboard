# lidar-slam-dashboard

Python/FastAPI dashboard that listens for telemetry, maps it into a live
occupancy grid, and serves a minimal browser UI. **Project 4** of a small
multi-repo lidar-mapping robot project I've been building. See
[`DATA_CONTRACT.md`](DATA_CONTRACT.md) for the wire format this app
parses and emits.

```
                    UDP :5005 (broadcast, receive)
base-radio  <==================================>  this app  ---->  browser
(lidarbase.local)   UDP :5006 (unicast, send)        |   (HTTP /status, /control,
                                                      |    WS /ws -> grid updates)
                                              occupancy grid
                                          (Bresenham ray-casting)
```

## Overview

`app/udp_listener.py` binds a UDP socket on `:5005` and reconstructs
sweeps the same way `lidar-android-app` does: `scan_sample` frames
accumulate into the current sweep's point list, and a `scan_complete`
frame hands the whole list to `app/occupancy_grid.py`, which ray-casts
every point into a 2D grid using `app/bresenham.py`'s hand-rolled
Bresenham line algorithm (free cells along the ray, an occupied cell at
the endpoint for valid in-range readings). FastAPI (`app/main.py`)
exposes the result over HTTP/WebSocket, and `app/control_client.py` sends
`control_command` frames to base-radio on unicast UDP `:5006`.

**v1 assumes a stationary scanning head.** There is no odometry, no pose
estimation, and no loop closure -- every sweep is ray-cast from the same
fixed grid-cell origin. This is deliberately *not* full SLAM; see
"Known limitations" below and `app/occupancy_grid.py`'s module docstring
for the reasoning. Moving-rover SLAM (odometry + pose estimation + loop
closure) would be the natural next step, but is out of scope for v1.

## SDK / runtime versions

- Python 3.11+
- FastAPI, uvicorn (with `[standard]` extras for the production ASGI
  loop), NumPy, Pydantic -- see `requirements.txt` for pinned ranges.

## Build & run instructions

```sh
cd lidar-slam-dashboard
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m app.main
```

Then open `http://localhost:8000/` in a browser. The server binds
`0.0.0.0:8000` for HTTP/WS and `0.0.0.0:5005` for the UDP telemetry
listener (the same process owns both -- see `app/main.py`'s `lifespan`
handler).

## Configuration

`app/control_client.py` resolves base-radio via the hostname
`lidarbase.local` using the OS's standard resolver
(`socket.gethostbyname`). This works out of the box on systems with
working mDNS-to-DNS bridging (Avahi on most Linux desktops, Bonjour on
macOS or Windows-with-Bonjour) but **not** on minimal servers without an
mDNS resolver installed. If resolution fails, `/control` returns
`502 Bad Gateway` with the exact fix: set the `LIDARBASE_HOST`
environment variable to base-radio's IP address directly, e.g.:

```sh
LIDARBASE_HOST=192.168.1.50 python -m app.main
```

(This is the one place this app's mDNS story differs from
`lidar-android-app`, which has no OS resolver to lean on and so
implements its own RFC 6762 client -- see that project's README.)

**Running the bot simulator instead of real hardware?** Point
`LIDARBASE_HOST` at wherever the simulator is listening, e.g.
`LIDARBASE_HOST=127.0.0.1` if it's running on the same machine/WSL
instance -- see [`simulator/README.md`](simulator/README.md).

## API surface

| Endpoint | Method | What |
|---|---|---|
| `/status` | GET | health snapshot, link/sequence-loss stats, grid dimensions + sweep count |
| `/control` | POST | JSON `{"cmd": "start_scan"\|"stop_scan"\|"set_sweep_range"\|"ping", "param1": int, "param2": int}` -> sends a `control_command` frame to base-radio |
| `/ws` | WebSocket | pushes `{"kind": "grid_update", "grid": {...}, "last_sweep_dir": int\|null}` once on connect and again after every completed sweep |
| `/` | GET | static frontend (`app/static/index.html` + `style.css` + `app.js`) |

## The occupancy grid mapper

- Grid: 300x300 cells at 2 cm/cell by default (`app/occupancy_grid.py`,
  `GridConfig`), sensor origin near the bottom-center cell.
- For each `scan_sample`, `OccupancyGrid.integrate_point()` converts
  `(angle_cdeg, distance_mm)` to a grid-cell endpoint and calls
  `bresenham_line()` (in `app/bresenham.py`, written from scratch -- no
  existing line-drawing/SLAM library) to get every cell on the ray.
  Cells before the endpoint are marked "free" (a miss counter
  increments); the endpoint cell is marked "occupied" (a hit counter
  increments) for valid in-range readings only.
- `OccupancyGrid.probability_grid()` exports `hits / (hits + misses)`
  per cell, with unobserved cells reported as `-1` (unknown) rather than
  `0` (observed-and-free) -- the frontend renders these differently.
- This hit/miss-ratio model is simpler than a Bayesian log-odds update
  (the classical Moravec/Elfes occupancy-grid-mapping formulation) by
  design for v1; see the module docstring for where that would extend.

## Architecture

![Dashboard data flow: UDP telemetry in, occupancy grid build, FastAPI/WebSocket out to the browser](docs/dashboard_data_flow.png)

End-to-end path from raw UDP frames to the rendered grid: `udp_listener.py`
parses incoming frames and reconstructs sweeps, `occupancy_grid.py`
ray-casts each sweep into a probability grid, `state.py` holds the
shared in-memory app state (grid + health + link stats), and
`main.py`/`ws_manager.py` push grid updates out over `/ws` to every
connected browser tab.

Example occupancy grid output, rendered from a synthetic sweep using the
same grid-to-image logic the live canvas uses (black/dark = occupied,
light = free, gray = unknown/unobserved):

![Example occupancy grid render: a room outline with a square pillar mapped from ray-cast hits and misses](docs/occupancy_grid_output.png)

## File structure

```
lidar-slam-dashboard/
├── app/
│   ├── main.py            FastAPI app, lifespan-managed UDP listener, routes
│   ├── contract.py        wire format: pack/unpack/CRC16/all 5 types
│   ├── udp_listener.py    asyncio UDP listener, sweep reconstruction
│   ├── occupancy_grid.py  hand-rolled occupancy grid (hit/miss ray-casting)
│   ├── bresenham.py       Bresenham's line algorithm, from scratch
│   ├── control_client.py  sends control_command via UDP unicast :5006
│   ├── state.py           shared AppState (grid, health, link stats)
│   ├── ws_manager.py      WebSocket connection tracking + broadcast
│   └── static/
│       ├── index.html     dark "mission control" UI shell
│       ├── style.css      theme: CSS-grid layout, link/telemetry status pills
│       └── app.js         canvas rendering, status polling, /control wiring
├── simulator/             bot simulators (no hardware needed) -- see below
│   ├── maps/room_with_pillar.map
│   ├── cpp/simulate_bot.cpp + Makefile
│   ├── java/SimulateBot.java
│   └── README.md
├── requirements.txt
├── .gitignore / LICENSE
```

## Frontend

`app/static/` is a dark, dashboard-style UI: a top bar with three live
status pills (browser&harr;server WebSocket state, dashboard&harr;bot
telemetry recency, and sweep count), a large occupancy-grid canvas with a
legend and "last sweep: forward/reverse" indicator, and a sidebar with
link stats, battery/fault health, sweep controls, and a scrolling
activity log. WebSocket connectivity and telemetry liveness are shown as
**two separate indicators** on purpose -- a browser tab can be happily
connected to the server while the bot/simulator itself has gone silent,
and collapsing those into one light would hide that distinction.

## Simulator (no hardware required)

[`simulator/`](simulator/) contains two independent, dependency-free bot
simulators -- one in C++, one in Java -- that ray-cast against a
predefined map (`simulator/maps/room_with_pillar.map`) and drive this
dashboard over the real UDP wire protocol, including responding to
`start_scan`/`stop_scan`/`set_sweep_range`/`ping` so the dashboard's
controls visibly change what gets mapped. See
[`simulator/README.md`](simulator/README.md) for build/run instructions
(WSL2-friendly) and the map format. This is the easiest way to see the
dashboard build a live map without any STM32/ESP32 hardware.

## Data contract types touched

All five types: `scan_sample` and `scan_complete` drive the occupancy
grid, `health_status` populates `/status`, `control_ack` is logged, and
`control_command` is the only type this app encodes/sends (via
`/control`).

## Known limitations

- **No odometry / stationary-head assumption** -- see "Overview" above.
  Moving the sensor between sweeps will silently corrupt the map (no
  detection or warning for this case in v1).
- **`LIDARBASE_HOST` mDNS fallback depends on the OS resolver** -- unlike
  `lidar-android-app`'s self-contained mDNS client, this app has no
  built-in raw-mDNS fallback; see "Configuration" above.
- **Single shared grid, single process** -- no per-client grids, no
  persistence across restarts (the grid is rebuilt from scratch in
  memory every time the app starts).
- **Quantized WS payload** (`int16` -1..100 per cell, see
  `to_serializable()`) trades a small amount of precision for a much
  smaller JSON payload; acceptable since the frontend only needs ~100
  shades of "free vs. occupied" to render usefully.
- **No authentication** on any endpoint -- this is a LAN dashboard, not
  intended to be exposed to the open internet.
- Written and tested against synthetic UDP datagrams crafted with
  `app/contract.py` itself, and against the two bundled simulators (see
  "Testing"), not against a live base-radio/bot-radio pair.

## Testing

This app **was** runnable and exercised end-to-end during development,
since FastAPI/uvicorn/NumPy run fine in a plain Python environment
(unlike Projects 1-3, which need physical/cross-compiled hardware). What
was verified directly:

- `app/contract.py`'s CRC16 implementation reproduces the standard
  CRC-16/CCITT-FALSE catalogue check value (`0x29B1` for ASCII
  `"123456789"`).
- `app/bresenham.py` produces correct cell sequences across all eight
  octants (verified against hand-checked coordinate pairs).
- `app/occupancy_grid.py` correctly accumulates hits/misses from a
  synthetic sweep.
- The full FastAPI app boots via `python -m app.main`, binds UDP `:5005`,
  and serves `/status` (200), `/` (200, static `index.html`), `/control`
  (returns a clear `502` with a remediation hint when `lidarbase.local`
  doesn't resolve, as it doesn't in this development environment), and `/ws` (accepts a
  connection and pushes an initial `grid_update`).
- A synthetic UDP client sending hand-crafted `scan_sample` +
  `scan_complete` frames to `127.0.0.1:5005` was used to confirm the
  listener parses frames, reconstructs a sweep, integrates it into the
  grid, and updates `/status`'s `sweeps_completed` counter end-to-end.
- Both `simulator/cpp/simulate_bot` and `simulator/java/SimulateBot` were
  built and run against this app (with
  `LIDARBASE_HOST=127.0.0.1` so `/control` resolves to the simulator
  instead of real hardware): telemetry flowed continuously with zero CRC
  failures, `/control`'s `stop_scan` measurably froze `frames_received`/
  `sweep_count` growth (only periodic `health_status` frames kept
  arriving), `start_scan` resumed it, and `ping`/`set_sweep_range` were
  accepted and acked by both simulators independently.

What was **not** verified: behavior against a real base-radio (no ESP32
hardware available yet) and real-world mDNS resolution of
`lidarbase.local` (no such host on this development network).

## License

MIT -- see [`LICENSE`](LICENSE).
