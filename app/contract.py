"""Canonical Python implementation of the lidar-bot-system wire data
contract.

Must stay byte-for-byte identical in behavior to:
  - stm32-lidar-firmware/Inc/data_contract.h (+ .c)
  - esp32-raw-mac-radio/common/data_contract.h (+ .c)
  - lidar-android-app .../data/LidarContract.kt

See DATA_CONTRACT.md at the repo root for the prose spec, including the
documented correction to the original spec: a frame is 16 bytes on the
wire (a 14-byte sof+type+seq+payload struct, plus a 2-byte big-endian
CRC16 trailer) -- "14 bytes" in the original spec refers only to the pre-CRC
struct.

CRC16 spec: CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, refin=false,
refout=false, xorout=0x0000), transmitted big-endian. Mismatched CRC
parameters or trailer byte order is the single most likely silent bug in
this whole system -- if every packet fails to parse, check this module
against the other three language implementations first.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

FRAME_LEN = 14   # sof + type + seq + payload, no CRC
WIRE_LEN = 16    # FRAME_LEN + 2-byte CRC trailer
PAYLOAD_LEN = 10

SOF_TELEMETRY = 0xAA
SOF_CONTROL = 0xAB

OUT_OF_RANGE = 0xFFFF


class Type:
    SCAN_SAMPLE = 0x01
    SCAN_COMPLETE = 0x02
    HEALTH_STATUS = 0x03
    CONTROL_COMMAND = 0x10
    CONTROL_ACK = 0x11


class CmdId:
    START_SCAN = 0x01
    STOP_SCAN = 0x02
    SET_SWEEP_RANGE = 0x03
    PING = 0x04


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, bit-by-bit -- matches the C/Kotlin
    implementations exactly (no table lookup) so there's only one
    algorithm to audit across all four languages."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


@dataclass
class Frame:
    sof: int
    type: int
    seq: int
    payload: bytes


def pack(sof: int, type_: int, seq: int, payload: bytes) -> bytes:
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload must be {PAYLOAD_LEN} bytes, got {len(payload)}")
    body = struct.pack("<BBH10s", sof, type_, seq, payload)
    assert len(body) == FRAME_LEN
    crc = crc16(body)
    return body + struct.pack(">H", crc)


def unpack(wire: bytes) -> Optional[Frame]:
    """Returns None if `wire` isn't exactly WIRE_LEN bytes or the CRC
    doesn't match. Callers must not act on a None result."""
    if len(wire) != WIRE_LEN:
        return None
    body = wire[:FRAME_LEN]
    expected = crc16(body)
    (received,) = struct.unpack(">H", wire[FRAME_LEN:])
    if expected != received:
        return None
    sof, type_, seq, payload = struct.unpack("<BBH10s", body)
    return Frame(sof=sof, type=type_, seq=seq, payload=payload)


@dataclass
class ScanSample:
    angle_cdeg: int
    distance_mm: int
    timestamp_ms: int


def decode_scan_sample(f: Frame) -> ScanSample:
    angle, dist, ts = struct.unpack("<HHI", f.payload[:8])
    return ScanSample(angle_cdeg=angle, distance_mm=dist, timestamp_ms=ts)


@dataclass
class ScanComplete:
    sweep_dir: int
    timestamp_ms: int


def decode_scan_complete(f: Frame) -> ScanComplete:
    sweep_dir = f.payload[0]
    (ts,) = struct.unpack("<I", f.payload[2:6])
    return ScanComplete(sweep_dir=sweep_dir, timestamp_ms=ts)


@dataclass
class HealthStatus:
    fault_flags: int
    battery_mv: int
    timestamp_ms: int


def decode_health_status(f: Frame) -> HealthStatus:
    flags, batt, ts = struct.unpack("<HHI", f.payload[:8])
    return HealthStatus(fault_flags=flags, battery_mv=batt, timestamp_ms=ts)


@dataclass
class ControlAck:
    cmd_id: int
    status: int
    timestamp_ms: int


def decode_control_ack(f: Frame) -> ControlAck:
    cmd_id = f.payload[0]
    status = f.payload[1]
    (ts,) = struct.unpack("<I", f.payload[6:10])
    return ControlAck(cmd_id=cmd_id, status=status, timestamp_ms=ts)


def encode_control_command(
    cmd_id: int,
    param1: int = 0,
    param2: int = 0,
    timestamp_ms: Optional[int] = None,
    seq: int = 0,
) -> bytes:
    """Builds a complete 16-byte control_command wire frame ready to send
    via UDP unicast to base-radio:5006. `seq` defaults to 0 -- like the
    Android app, the dashboard does not maintain its own outbound
    sequence space; neither base-radio nor bot-radio reject frames on
    `seq` (informational for loss-tracking only)."""
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF
    payload = struct.pack("<BBHHI", cmd_id, 0, param1, param2, timestamp_ms)
    return pack(SOF_CONTROL, Type.CONTROL_COMMAND, seq, payload)
