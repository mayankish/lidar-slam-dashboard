# Data contract

The single source of truth for the wire format used across every project
in this multi-repo lidar-mapping system. If any implementation disagrees
with this document, the implementation is wrong (or this document is
stale and needs fixing) -- not the other way around. Four independent
implementations exist and must stay byte-for-byte identical:

| Language | File |
|---|---|
| C (bare-metal) | [`stm32-lidar-firmware/Inc/data_contract.h`](https://github.com/mayankish/stm32-lidar-firmware/blob/main/Inc/data_contract.h) + [`Src/data_contract.c`](https://github.com/mayankish/stm32-lidar-firmware/blob/main/Src/data_contract.c) |
| C (ESP-IDF) | [`esp32-raw-mac-radio/common/data_contract.h`](https://github.com/mayankish/esp32-raw-mac-radio/blob/main/common/data_contract.h) + `.c` (duplicated into `bot-radio/main/` and `base-radio/main/` -- see that project's README) |
| Kotlin | [`lidar-android-app/.../data/LidarContract.kt`](https://github.com/mayankish/lidar-android-app/blob/main/app/src/main/java/com/lidarbotsystem/app/data/LidarContract.kt) |
| Python | [`app/contract.py`](app/contract.py) |

## Resolved ambiguity notes

The original design notes for this system described the contract in
terms that needed two clarifications during implementation, made here
explicitly rather than silently:

1. **"Fixed 14-byte frame" + "crc16 over bytes 0-11"** -- taken literally,
   these two statements are inconsistent (a 14-byte frame has bytes
   0-13; "bytes 0-11" would only cover 12 of them, excluding 2 payload
   bytes from the CRC for no documented reason). The implemented
   resolution: **the struct (`sof`+`type`+`seq`+`payload`) is 14 bytes
   (offsets 0-13), the CRC is computed over the full 14 bytes (0-13),
   and the CRC itself is appended as 2 more bytes (14-15) -- making 16
   bytes the actual size of everything that goes over a wire.** "14-byte
   frame" in the original notes is read as describing the pre-CRC struct,
   not the total wire size. This is called out in every implementation's
   header/module comment so nobody changes one without rereading this
   section.
2. **base-radio (`esp32-raw-mac-radio/base-radio`) broadcasts every frame
   type it receives over the raw link onto UDP `:5005`**, including
   `control_ack` -- not only the three "telemetry" types
   (`scan_sample`/`scan_complete`/`health_status`) the original prose
   data-flow description names as the telemetry path. This dashboard's
   `app/udp_listener.py` therefore dispatches on `type` for all five
   frame values rather than assuming only three will ever arrive on
   `:5005`. See `esp32-raw-mac-radio/base-radio/README.md`'s "Why
   broadcast everything" section for the full reasoning.

## Wire format

```
byte:    0     1     2     3     4 .. 13        14    15
field: [sof] [type] [-- seq --] [-- payload --] [--  crc16  --]
              (u8)   (u16, LE)     (10 bytes)      (u16, BE)
```

- **`sof`** (1 byte): start-of-frame marker.
  - `0xAA` -- telemetry direction (bot -> base station): `scan_sample`,
    `scan_complete`, `health_status`, `control_ack`.
  - `0xAB` -- control direction (base station -> bot): `control_command`.
  - Note that `control_ack` flows in the telemetry direction (bot ->
    base) and so uses `sof = 0xAA`, even though it's conceptually a
    response to a control command -- it's grouped by *direction of
    travel*, not by which "side" of the conversation initiated it.
- **`type`** (1 byte): see "Packet types" below.
- **`seq`** (2 bytes, **little-endian**): a single monotonically
  increasing counter, shared across every frame type the STM32 sends --
  there is no per-type sequence space. Used for loss detection only
  (see this repo's `app/state.py`'s `LinkStats.track_sequence()`, which
  mirrors the equivalent logic in `esp32-raw-mac-radio/base-radio`);
  never used to reject or reorder frames.
- **`payload`** (10 bytes): interpretation depends on `type` -- see
  below. Unused trailing bytes within a payload are always written as
  `0x00` by every encoder (never left uninitialized), so payloads are
  deterministic and diffable even though the decoders don't read those
  bytes.
- **`crc16`** (2 bytes, **big-endian**, deliberately the opposite byte
  order of `seq`): CRC-16/CCITT-FALSE over bytes 0-13 (the `sof` through
  `payload` bytes, i.e. everything except the CRC itself).
  - poly = `0x1021`, init = `0xFFFF`, refin = `false`, refout = `false`,
    xorout = `0x0000`.
  - Catalogue check value: `crc16("123456789") == 0x29B1` -- this
    repo's `app/contract.py` is unit-tested against this value, same as
    every other implementation in the system (see "Testing" in the
    main README).
  - **This is the single most likely silent bug in this whole system.**
    If telemetry looks garbled, or every packet fails CRC, check the
    CRC parameters and the trailer byte order against this document
    before anything else.

## Packet types

| `type` | Name | Direction | `sof` |
|---|---|---|---|
| `0x01` | `scan_sample` | bot -> base | `0xAA` |
| `0x02` | `scan_complete` | bot -> base | `0xAA` |
| `0x03` | `health_status` | bot -> base | `0xAA` |
| `0x10` | `control_command` | base -> bot | `0xAB` |
| `0x11` | `control_ack` | bot -> base | `0xAA` |

### `scan_sample` (0x01) payload

| offset | size | field | notes |
|---|---|---|---|
| 0 | 2 | `angle_cdeg` | u16 LE, centidegrees (e.g. `9000` = 90.00°) |
| 2 | 2 | `distance_mm` | u16 LE, millimeters; `0xFFFF` = out of range |
| 4 | 4 | `timestamp_ms` | u32 LE, milliseconds since boot |
| 8 | 2 | (reserved) | always `0x0000` |

This dashboard accumulates `scan_sample`s (via `OccupancyGrid.integrate_point()`
in `app/occupancy_grid.py`) until a `scan_complete` arrives.

### `scan_complete` (0x02) payload

| offset | size | field | notes |
|---|---|---|---|
| 0 | 1 | `sweep_dir` | `0` = forward, `1` = reverse |
| 1 | 1 | (reserved) | always `0x00` |
| 2 | 4 | `timestamp_ms` | u32 LE |
| 6 | 4 | (reserved) | always `0x00000000` |

Marks the end of one sweep -- this dashboard uses it to flush/finalize
the accumulated `scan_sample`s as one sweep (`OccupancyGrid.integrate_sweep()`)
and broadcast the updated grid to connected browser clients over `/ws`.

### `health_status` (0x03) payload

| offset | size | field | notes |
|---|---|---|---|
| 0 | 2 | `fault_flags` | u16 LE, bitfield |
| 2 | 2 | `battery_mv` | u16 LE, millivolts |
| 4 | 4 | `timestamp_ms` | u32 LE |
| 8 | 2 | (reserved) | always `0x0000` |

Surfaced by this dashboard's `/status` endpoint (`app/state.py`'s
`HealthSnapshot`) and rendered as the battery bar / fault pills in the
browser UI.

### `control_command` (0x10) payload

| offset | size | field | notes |
|---|---|---|---|
| 0 | 1 | `cmd_id` | see "Command IDs" below |
| 1 | 1 | (reserved) | always `0x00` |
| 2 | 2 | `param1` | u16 LE, meaning depends on `cmd_id` |
| 4 | 2 | `param2` | u16 LE, meaning depends on `cmd_id` |
| 6 | 4 | `timestamp_ms` | u32 LE, sender's clock |

This is the one frame type this dashboard *sends* rather than receives
-- see `app/control_client.py`'s `send_control_command()`, called from
the `/control` POST route in `app/main.py`.

### `control_ack` (0x11) payload

| offset | size | field | notes |
|---|---|---|---|
| 0 | 1 | `cmd_id` | echoes the command being acknowledged |
| 1 | 1 | `status` | `0` = ok, `1` = error |
| 2 | 4 | (reserved) | always `0x00000000` |
| 6 | 4 | `timestamp_ms` | u32 LE, bot's clock at ack time |

## Command IDs (`control_command.cmd_id`)

| `cmd_id` | Name | `param1` | `param2` |
|---|---|---|---|
| `0x01` | `START_SCAN` | unused (0) | unused (0) |
| `0x02` | `STOP_SCAN` | unused (0) | unused (0) |
| `0x03` | `SET_SWEEP_RANGE` | `min_angle_cdeg` | `max_angle_cdeg` |
| `0x04` | `PING` | unused (0) | unused (0) |

These are exposed as the four buttons in the browser UI (`app/static/app.js`'s
`sendControl()`), encoded via `app/contract.py`'s `encode_control_command()`.

## End-to-end data flow

```
   STM32 (bare-metal FreeRTOS)              ESP32 #1 "bot-radio"
   stm32-lidar-firmware                     esp32-raw-mac-radio/bot-radio
  +-------------------------+   UART1      +---------------------------+
  | StepperTask SensorTask  |  16-byte     | UART RX -> raw_link_send() |
  | TelemetryTask HealthTask|  wire frames | raw_link RX -> UART TX     |
  | command_handler         | <==========> |                           |
  +-------------------------+  115200 8N1  +---------------------------+
                                                       ^  |
                                          raw 802.11,  |  |  fixed channel,
                                       no association, |  |  promiscuous RX /
                                        OUI-tagged      |  |  esp_wifi_80211_tx()
                                       action frames    |  v
                                            +---------------------------+
                                            | ESP32 #2 "base-radio"     |
                                            | esp32-raw-mac-radio/base- |
                                            | radio: Wi-Fi STA, mDNS    |
                                            | lidarbase.local           |
                                            +---------------------------+
                                                  |               ^
                                     UDP :5005    |               | UDP :5006
                                     broadcast    |               | unicast
                                  (every frame    |               | (control_command
                                   type)          v               |  only)
                          +------------------+         +------------------+
                          | lidar-android-app|         | lidar-slam-      |
                          | (Kotlin/Compose) |         | dashboard (this  |
                          |                  |         | repo, FastAPI)   |
                          +------------------+         +------------------+
```

This repo's `app/udp_listener.py` binds UDP `:5005` and is one of two
independent clients/peers of base-radio -- it does not talk to
`lidar-android-app` directly, and vice versa. Outbound `control_command`
frames go from this repo's `app/control_client.py` to base-radio's UDP
`:5006`, resolved via mDNS (`lidarbase.local`) with a `LIDARBASE_HOST`
environment-variable fallback for networks without mDNS.

## Known operational caveat: ESP32 channel lock

`esp32-raw-mac-radio/base-radio` is the only firmware in this system that
joins a real Wi-Fi network. Once it associates with an AP, the ESP32's
single radio locks onto the AP's channel -- which also governs the raw
link's promiscuous-mode RX, regardless of the `RAW_LINK_CHANNEL`
constant configured before association. **The fix is operational, not a
code fix: configure your Wi-Fi AP's channel to match `RAW_LINK_CHANNEL`.**
This doesn't affect this dashboard directly, but it's the most common
reason "telemetry just stops arriving on `:5005`" during setup -- see
`esp32-raw-mac-radio`'s READMEs for the full explanation before assuming
a bug in this repo's listener.
