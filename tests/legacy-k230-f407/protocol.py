"""K230 <-> F407 protocol pack/unpack helpers.

This module mirrors the frame definition in K230_F407_通信协议_v1.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Optional

SOF1 = 0xAA
SOF2 = 0x55


class FrameType(IntEnum):
    OBS = 0x01
    HEARTBEAT = 0x02
    PARAM = 0x10
    FEEDBACK = 0x81
    ACK = 0x90


class ParamCmd(IntEnum):
    SET_PID = 0x01
    SET_STATE = 0x02
    SET_ZERO = 0x03


OBS_STRUCT = struct.Struct("<IhhhhBBBB")
HB_STRUCT = struct.Struct("<IH")
PARAM_STRUCT = struct.Struct("<Bhhh")
FB_STRUCT = struct.Struct("<BHHHH")


@dataclass
class ObsPayload:
    timestamp_ms: int
    center_u: int
    center_v: int
    err_u: int
    err_v: int
    confidence: int
    target_lost: int
    mode_hint: int = 0
    reserved: int = 0

    def to_bytes(self) -> bytes:
        err_u = self.err_u
        err_v = self.err_v
        confidence = self.confidence
        if self.target_lost:
            err_u = 0
            err_v = 0
            confidence = 0

        return OBS_STRUCT.pack(
            self.timestamp_ms & 0xFFFFFFFF,
            int(self.center_u),
            int(self.center_v),
            int(err_u),
            int(err_v),
            int(max(0, min(confidence, 100))),
            1 if self.target_lost else 0,
            int(self.mode_hint) & 0xFF,
            int(self.reserved) & 0xFF,
        )


@dataclass
class HeartbeatPayload:
    timestamp_ms: int
    status_bits: int

    def to_bytes(self) -> bytes:
        return HB_STRUCT.pack(self.timestamp_ms & 0xFFFFFFFF, self.status_bits & 0xFFFF)


@dataclass
class ParamPayload:
    cmd_id: int
    arg0: int = 0
    arg1: int = 0
    arg2: int = 0

    def to_bytes(self) -> bytes:
        return PARAM_STRUCT.pack(self.cmd_id & 0xFF, int(self.arg0), int(self.arg1), int(self.arg2))


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def pack_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    head = bytes([SOF1, SOF2, frame_type & 0xFF, seq & 0xFF, len(payload) & 0xFF])
    body = bytes([head[2], head[3], head[4]]) + payload
    cs = checksum(body)
    return head + payload + bytes([cs])


def pack_obs(seq: int, payload: ObsPayload) -> bytes:
    return pack_frame(FrameType.OBS, seq, payload.to_bytes())


def pack_heartbeat(seq: int, payload: HeartbeatPayload) -> bytes:
    return pack_frame(FrameType.HEARTBEAT, seq, payload.to_bytes())


def pack_param(seq: int, payload: ParamPayload) -> bytes:
    return pack_frame(FrameType.PARAM, seq, payload.to_bytes())


@dataclass
class DecodedFrame:
    frame_type: int
    seq: int
    payload: bytes


class StreamParser:
    """Streaming byte parser matching gimbal_proto.c state machine."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = 0
        self.frame_type = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()
        self.running_sum = 0

    def push(self, byte: int) -> Optional[DecodedFrame]:
        b = byte & 0xFF

        if self.state == 0:  # wait SOF1
            if b == SOF1:
                self.state = 1
            return None

        if self.state == 1:  # wait SOF2
            if b == SOF2:
                self.state = 2
                self.running_sum = 0
            elif b != SOF1:
                self.state = 0
            return None

        if self.state == 2:  # type
            self.frame_type = b
            self.running_sum = b
            self.state = 3
            return None

        if self.state == 3:  # seq
            self.seq = b
            self.running_sum = (self.running_sum + b) & 0xFF
            self.state = 4
            return None

        if self.state == 4:  # len
            self.length = b
            self.running_sum = (self.running_sum + b) & 0xFF
            self.payload.clear()
            if self.length == 0:
                self.state = 6
            else:
                self.state = 5
            return None

        if self.state == 5:  # payload
            self.payload.append(b)
            self.running_sum = (self.running_sum + b) & 0xFF
            if len(self.payload) >= self.length:
                self.state = 6
            return None

        if self.state == 6:  # checksum
            ok = (self.running_sum & 0xFF) == b
            if ok:
                out = DecodedFrame(self.frame_type, self.seq, bytes(self.payload))
            else:
                out = None
            self.reset()
            return out

        self.reset()
        return None


def decode_feedback(payload: bytes) -> Optional[dict]:
    if len(payload) != FB_STRUCT.size:
        return None
    ctrl_state, yaw_pwm, pitch_pwm, fault_code, lap_phase_q15 = FB_STRUCT.unpack(payload)
    return {
        "ctrl_state": ctrl_state,
        "yaw_pwm": yaw_pwm,
        "pitch_pwm": pitch_pwm,
        "fault_code": fault_code,
        "lap_phase_q15": lap_phase_q15,
    }


def decode_ack(payload: bytes) -> Optional[dict]:
    if len(payload) < 1:
        return None
    return {
        "ack_code": payload[0],
        "raw": payload,
    }

