"""K230/CPython implementation of the vision link protocol v1.

The module intentionally uses only small, commonly available Python features so
that it can run on CanMV MicroPython as well as desktop CPython.
"""

import struct

SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
MAX_PAYLOAD = 64

TYPE_VISION_OBSERVATION = 0x01
TYPE_HEARTBEAT = 0x02
TYPE_COMMAND = 0x10
TYPE_ACK = 0x90

FLAG_TARGET_VALID = 1 << 0
FLAG_RESULT_STABLE = 1 << 1
FLAG_VALUE_SATURATED = 1 << 2
FLAG_PROCESSING_DEGRADED = 1 << 3

STATUS_OK = 0
STATUS_UNSUPPORTED = 1
STATUS_BAD_PAYLOAD = 2
STATUS_BAD_STATE = 3
STATUS_BUSY = 4
STATUS_INTERNAL_ERROR = 5


def _crc_update(crc, byte):
    crc ^= (byte & 0xFF) << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc = _crc_update(crc, byte)
    return crc


def pack_frame(message_type, seq, payload=b""):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((VERSION, message_type & 0xFF, seq & 0xFF, len(payload))) + payload
    crc = crc16_ccitt_false(body)
    return bytes((SOF1, SOF2)) + body + struct.pack("<H", crc)


def pack_observation(seq, timestamp_ms, center_x, center_y, error_x, error_y,
                     confidence, flags, target_id=0, mode=0):
    flags &= 0x0F
    if not (flags & FLAG_TARGET_VALID):
        center_x = 0
        center_y = 0
        error_x = 0
        error_y = 0
        confidence = 0
        target_id = 0xFF
    confidence = max(0, min(int(confidence), 100))
    payload = struct.pack(
        "<IhhhhBBBB",
        int(timestamp_ms) & 0xFFFFFFFF,
        int(center_x),
        int(center_y),
        int(error_x),
        int(error_y),
        confidence,
        flags,
        int(target_id) & 0xFF,
        int(mode) & 0xFF,
    )
    return pack_frame(TYPE_VISION_OBSERVATION, seq, payload)


def pack_heartbeat(seq, timestamp_ms, status_bits, fps_x10):
    payload = struct.pack(
        "<IHH",
        int(timestamp_ms) & 0xFFFFFFFF,
        int(status_bits) & 0xFFFF,
        int(fps_x10) & 0xFFFF,
    )
    return pack_frame(TYPE_HEARTBEAT, seq, payload)


def pack_command(seq, command_id, mode=0, arg0=0, arg1=0, arg2=0):
    payload = struct.pack(
        "<BBhhh",
        int(command_id) & 0xFF,
        int(mode) & 0xFF,
        int(arg0),
        int(arg1),
        int(arg2),
    )
    return pack_frame(TYPE_COMMAND, seq, payload)


def pack_ack(seq, request_type, request_seq, status=STATUS_OK, detail=0):
    payload = bytes((
        int(request_type) & 0xFF,
        int(request_seq) & 0xFF,
        int(status) & 0xFF,
        int(detail) & 0xFF,
    ))
    return pack_frame(TYPE_ACK, seq, payload)


def decode_observation(payload):
    if len(payload) != 16:
        return None
    values = struct.unpack("<IhhhhBBBB", payload)
    return {
        "timestamp_ms": values[0],
        "center_x": values[1],
        "center_y": values[2],
        "error_x": values[3],
        "error_y": values[4],
        "confidence": values[5],
        "flags": values[6],
        "target_id": values[7],
        "mode": values[8],
    }


def decode_command(payload):
    if len(payload) != 8:
        return None
    values = struct.unpack("<BBhhh", payload)
    return {
        "command_id": values[0],
        "mode": values[1],
        "arg0": values[2],
        "arg1": values[3],
        "arg2": values[4],
    }


class StreamParser:
    WAIT_SOF1 = 0
    WAIT_SOF2 = 1
    READ_VERSION = 2
    READ_TYPE = 3
    READ_SEQ = 4
    READ_LENGTH = 5
    READ_PAYLOAD = 6
    READ_CRC_LOW = 7
    READ_CRC_HIGH = 8

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = self.WAIT_SOF1
        self.version = 0
        self.message_type = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()
        self.crc = 0xFFFF
        self.received_crc = 0

    def timeout(self):
        self.reset()

    def push(self, byte):
        byte &= 0xFF

        if self.state == self.WAIT_SOF1:
            if byte == SOF1:
                self.state = self.WAIT_SOF2
            return None

        if self.state == self.WAIT_SOF2:
            if byte == SOF2:
                self.state = self.READ_VERSION
                self.crc = 0xFFFF
            elif byte != SOF1:
                self.state = self.WAIT_SOF1
            return None

        if self.state == self.READ_VERSION:
            if byte != VERSION:
                self.reset()
                if byte == SOF1:
                    self.state = self.WAIT_SOF2
                return None
            self.version = byte
            self.crc = _crc_update(self.crc, byte)
            self.state = self.READ_TYPE
            return None

        if self.state == self.READ_TYPE:
            self.message_type = byte
            self.crc = _crc_update(self.crc, byte)
            self.state = self.READ_SEQ
            return None

        if self.state == self.READ_SEQ:
            self.seq = byte
            self.crc = _crc_update(self.crc, byte)
            self.state = self.READ_LENGTH
            return None

        if self.state == self.READ_LENGTH:
            self.length = byte
            self.crc = _crc_update(self.crc, byte)
            self.payload = bytearray()
            if self.length > MAX_PAYLOAD:
                self.reset()
            elif self.length == 0:
                self.state = self.READ_CRC_LOW
            else:
                self.state = self.READ_PAYLOAD
            return None

        if self.state == self.READ_PAYLOAD:
            self.payload.append(byte)
            self.crc = _crc_update(self.crc, byte)
            if len(self.payload) == self.length:
                self.state = self.READ_CRC_LOW
            return None

        if self.state == self.READ_CRC_LOW:
            self.received_crc = byte
            self.state = self.READ_CRC_HIGH
            return None

        if self.state == self.READ_CRC_HIGH:
            self.received_crc |= byte << 8
            if self.received_crc == self.crc:
                frame = (self.version, self.message_type, self.seq, bytes(self.payload))
            else:
                frame = None
            self.reset()
            return frame

        self.reset()
        return None
