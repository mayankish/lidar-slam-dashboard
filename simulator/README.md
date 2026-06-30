# Bot simulator

Two independent, from-scratch simulators that stand in for the real
`stm32-lidar-firmware` + `esp32-raw-mac-radio` hardware chain so the
dashboard can be driven and watched without any physical bot. Both
ray-cast a sweeping LIDAR head against the same predefined room layout
(`maps/room_with_pillar.map`) and speak `/DATA_CONTRACT.md`'s exact wire
format over UDP -- the dashboard cannot tell either of them apart from
real hardware.

Deliberately implemented twice, in two different languages, with no
shared code between them (or with the Python dashboard): that's the same
"independent re-implementation of one byte-for-byte contract per language
boundary" discipline the rest of this repo uses, now applied within this
project's own simulator/ folder.

```
simulator/
├── maps/
│   └── room_with_pillar.map   predefined room layout, shared by both simulators
├── cpp/
│   ├── simulate_bot.cpp       C++ simulator (POSIX sockets, no deps)
│   └── Makefile
└── java/
    └── SimulateBot.java       Java simulator (java.net.DatagramSocket)
```

## What it does

Each simulator, once running:

1. Loads `maps/room_with_pillar.map` (or a path you pass in).
2. Sweeps a virtual sensor head back and forth across 0-180°, ray-casting
   against the map's walls at each step and sending a `scan_sample` UDP
   frame per angle, plus a `scan_complete` frame at each end of the sweep.
3. Sends a synthetic `health_status` frame every couple of seconds
   (battery voltage drifts down then back up on a repeating cycle --
   cosmetic only, not a real discharge model).
4. Listens for `control_command` frames on its own UDP port and replies
   with `control_ack` -- `start_scan` and `stop_scan` actually pause/resume
   sample emission, `set_sweep_range` actually changes the swept angles,
   and `ping` just acks. This is what makes the dashboard's buttons
   visibly change what's being mapped.

Point either simulator at a running `lidar-slam-dashboard` and you'll see
the map build up live in the browser, exactly as if a real bot were
sweeping the room.

## The predefined map

`maps/room_with_pillar.map` is a small plaintext format (not JSON, so the
C++ simulator has zero parsing dependencies):

```
SENSOR_MAX_RANGE_MM <int>
WALL <x1> <y1> <x2> <y2>
```

The sensor is always the origin `(0, 0)`; `+x` is angle 0° (right), `+y`
is angle 90° (straight ahead), `-x` is angle 180° (left) -- this matches
`app/occupancy_grid.py`'s ray-casting convention exactly. The shipped map
is a ~2.4m x 1.6m room (open on the sensor's side) with one square pillar
near the middle; the far corners sit right at the map's 2000mm max range,
so a few extreme-corner readings legitimately come back out-of-range --
that's intentional, not a bug. See the comments at the top of the `.map`
file itself for the full, authoritative format spec.

Edit this file (or point a simulator at your own copy with the map-path
argument) to simulate a different room.

## Building and running

Both simulators are plain command-line tools with no external
dependencies. **Recommended: run everything (dashboard + simulator)
inside the same WSL2 instance** -- it avoids any Windows-host-to-WSL2
networking/firewall complications, since `localhost` then means the same
network namespace for both. From PowerShell or Windows Terminal:

```powershell
wsl
```

### C++

```sh
sudo apt install g++ make      # if not already present
cd lidar-slam-dashboard/simulator/cpp
make
./simulate_bot
```

### Java

```sh
sudo apt install default-jdk   # if not already present
cd lidar-slam-dashboard/simulator/java
javac SimulateBot.java
java SimulateBot
```

### Command-line options (identical for both)

```
[map_file]                       default: ../maps/room_with_pillar.map
--host=127.0.0.1                 dashboard's IP (where telemetry is sent)
--port=5005                      dashboard's telemetry UDP port
--listen-port=5006               this simulator's control_command UDP port
--step-deg=0.5                   degrees advanced per scan_sample
--sample-interval-ms=20          ms between scan_sample frames
--health-interval-ms=2000        ms between health_status frames
```

Examples:

```sh
./simulate_bot                                   # defaults, talks to localhost
./simulate_bot --step-deg=1.0 --sample-interval-ms=10   # faster, coarser sweep
java SimulateBot ../maps/room_with_pillar.map --host=192.168.1.20
```

**Run only one simulator at a time** unless you change `--port`/
`--listen-port` for one of them -- both default to the same dashboard
target (`:5005`) and the same control-listen port (`:5006`), so two
instances would fight over the same UDP socket.

## End-to-end: dashboard + simulator together

In one WSL terminal:

```sh
cd lidar-slam-dashboard
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m app.main
```

In a second WSL terminal:

```sh
cd lidar-slam-dashboard/simulator/cpp
make && ./simulate_bot
```

Open `http://localhost:8000/` in a Windows browser (WSL2 forwards
`localhost` to its own network namespace by default on recent Windows
11 builds). The map should start filling in within a couple of seconds.
Use the dashboard's Start/Stop/Ping/Set Range controls and watch the
simulator's terminal log each command and the dashboard's map respond
(pausing, resuming, or sweeping a different angular range).

## Protocol notes specific to the simulator

- `control_ack` is sent back to the dashboard's **telemetry** address
  (`--host:--port`, i.e. `:5005`), not to whatever address the
  `control_command` happened to arrive from -- this mirrors
  `/DATA_CONTRACT.md`'s rule that acks flow in the telemetry direction in
  the real base-radio/bot-radio system, not as a same-socket
  request/response.
- A `control_command` that fails CRC or has the wrong `sof`/`type` is
  dropped silently (logged to stderr) rather than acked, since there's no
  well-formed `cmd_id` to ack.
- `set_sweep_range` clamps to `[0, 180]` degrees and rejects (`status=1`
  in the ack) if `min >= max`.
