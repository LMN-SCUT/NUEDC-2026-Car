"""K230 -> F407 protocol-v1 one-file UART smoke test.

Run this file directly in CanMV IDE. No extra Python module is required.
It sends synthetic observations at 30 Hz and heartbeat at 2 Hz. When fault
injection is enabled, it performs one malformed-stream test and then resumes
normal traffic automatically.
"""

import math
import struct
import time

from machine import UART

try:
    from machine import FPIOA
except Exception:
    FPIOA = None


UART_ID = 3
TX_PIN = 32
RX_PIN = 33
BAUD = 115200

# One-shot parser recovery test. Set to False after acceptance testing.
FAULT_INJECTION_ENABLED = False
FAULT_START_DELAY_MS = 3000
FAULT_STEP_INTERVAL_MS = 1000

SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
TYPE_VISION_OBSERVATION = 0x01
TYPE_HEARTBEAT = 0x02
TYPE_COMMAND = 0x10
TYPE_ACK = 0x90
FLAG_TARGET_VALID = 1 << 0
FLAG_RESULT_STABLE = 1 << 1

CMD_SET_MODE = 0x01
CMD_START_STREAM = 0x02
CMD_STOP_STREAM = 0x03
CMD_SET_REFERENCE = 0x04

STATUS_OK = 0
STATUS_UNSUPPORTED = 1
STATUS_BAD_PAYLOAD = 2


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000)


def ticks_diff(new, old):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new, old)
    return new - old


def sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_frame(message_type, seq, payload):
    body = bytes((VERSION, message_type & 0xFF, seq & 0xFF, len(payload))) + payload
    crc = crc16_ccitt_false(body)
    return bytes((SOF1, SOF2)) + body + struct.pack("<H", crc)


def pack_observation(seq, timestamp_ms, center_x, center_y, error_x, error_y, mode):
    payload = struct.pack(
        "<IhhhhBBBB",
        timestamp_ms & 0xFFFFFFFF,
        center_x,
        center_y,
        error_x,
        error_y,
        90,
        FLAG_TARGET_VALID | FLAG_RESULT_STABLE,
        0,
        mode,
    )
    return pack_frame(TYPE_VISION_OBSERVATION, seq, payload)


def pack_heartbeat(seq, timestamp_ms):
    payload = struct.pack("<IHH", timestamp_ms & 0xFFFFFFFF, 0x0003, 300)
    return pack_frame(TYPE_HEARTBEAT, seq, payload)


def pack_ack(seq, request_seq, status, detail=0):
    payload = bytes((TYPE_COMMAND, request_seq & 0xFF, status & 0xFF, detail & 0xFF))
    return pack_frame(TYPE_ACK, seq, payload)


def inject_fault_step(uart_device, step, seq_value, now_ms, current_mode):
    """Inject one controlled stream fault and return the next sequence value."""
    if step == 0:
        # Includes a lone 0xAA to exercise SOF resynchronization.
        uart_device.write(bytes((0x00, 0xFF, 0x12, SOF1, 0x33, 0x7E)))
        print("[VL][FAULT] 1/4 noise bytes injected")
    elif step == 1:
        frame = pack_heartbeat(seq_value, now_ms)
        uart_device.write(frame[:-1] + bytes((frame[-1] ^ 0x5A,)))
        seq_value = (seq_value + 1) & 0xFF
        print("[VL][FAULT] 2/4 bad CRC frame injected")
    elif step == 2:
        # A length of 65 exceeds protocol-v1's 64-byte payload limit.
        uart_device.write(bytes((SOF1, SOF2, VERSION, TYPE_HEARTBEAT,
                                 seq_value, 65)))
        seq_value = (seq_value + 1) & 0xFF
        print("[VL][FAULT] 3/4 invalid length header injected")
    elif step == 3:
        frame = pack_observation(
            seq_value, now_ms, 320, 240, 0, 0, current_mode
        )
        uart_device.write(frame[:10])
        seq_value = (seq_value + 1) & 0xFF
        print("[VL][FAULT] 4/4 truncated frame injected; pausing 30 ms")
        sleep_ms(30)
        print("[VL][FAULT] sequence complete; normal traffic resumed")
    return seq_value


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
    def __init__(self):
        self.buffer = bytearray()

    def push(self, byte):
        self.buffer.append(byte & 0xFF)

        while len(self.buffer) >= 2:
            if self.buffer[0] == SOF1 and self.buffer[1] == SOF2:
                break
            self.buffer = self.buffer[1:]

        if len(self.buffer) < 6:
            return None

        if self.buffer[2] != VERSION:
            self.buffer = self.buffer[1:]
            return None

        payload_length = self.buffer[5]
        if payload_length > 64:
            self.buffer = self.buffer[1:]
            return None

        total_length = 8 + payload_length
        if len(self.buffer) < total_length:
            return None

        frame = bytes(self.buffer[:total_length])
        self.buffer = self.buffer[total_length:]
        received_crc = frame[-2] | (frame[-1] << 8)
        if crc16_ccitt_false(frame[2:-2]) != received_crc:
            return None
        return frame[3], frame[4], frame[6:-2]


def init_uart():
    # CanMV K230 may require explicit FPIOA mapping. Older firmware accepts
    # tx/rx directly in the UART constructor, so both forms are supported.
    if FPIOA is not None:
        try:
            fpioa = FPIOA()
            fpioa.set_function(TX_PIN, getattr(FPIOA, "UART{}_TXD".format(UART_ID)))
            fpioa.set_function(RX_PIN, getattr(FPIOA, "UART{}_RXD".format(UART_ID)))
        except Exception as exc:
            print("[VL] FPIOA mapping skipped:", exc)

    try:
        return UART(UART_ID, BAUD, tx=TX_PIN, rx=RX_PIN)
    except Exception:
        return UART(UART_ID, BAUD)


uart = init_uart()
seq = 0
parser = StreamParser()
stream_enabled = True
mode = 0
reference_x = 320
reference_y = 240
last_request_seq = None
last_ack_frame = None
last_observation_ms = ticks_ms()
last_heartbeat_ms = last_observation_ms
last_print_ms = last_observation_ms
fault_start_ms = last_observation_ms
last_fault_ms = last_observation_ms
fault_step = 0

print("[VL] standalone smoke sender")
print("[VL] UART{} TX=GPIO{} RX=GPIO{} {} 8N1".format(
    UART_ID, TX_PIN, RX_PIN, BAUD
))

while True:
    now = ticks_ms()

    if (FAULT_INJECTION_ENABLED and fault_step < 4 and
            ticks_diff(now, fault_start_ms) >= FAULT_START_DELAY_MS and
            ticks_diff(now, last_fault_ms) >= FAULT_STEP_INTERVAL_MS):
        seq = inject_fault_step(uart, fault_step, seq, now, mode)
        fault_step += 1
        last_fault_ms = now

    if stream_enabled and ticks_diff(now, last_observation_ms) >= 33:
        phase = now / 1000.0
        center_x = reference_x + int(40 * math.sin(phase))
        center_y = reference_y + int(30 * math.cos(phase * 0.8))
        uart.write(pack_observation(
            seq,
            now,
            center_x,
            center_y,
            center_x - reference_x,
            center_y - reference_y,
            mode,
        ))
        seq = (seq + 1) & 0xFF
        last_observation_ms = now

    if ticks_diff(now, last_heartbeat_ms) >= 500:
        uart.write(pack_heartbeat(seq, now))
        seq = (seq + 1) & 0xFF
        last_heartbeat_ms = now

    try:
        available = uart.any()
    except Exception:
        available = 0

    if available:
        data = uart.read(available)
        if data:
            for byte in data:
                received = parser.push(byte)
                if received is None or received[0] != TYPE_COMMAND:
                    continue

                request_seq = received[1]
                if last_request_seq == request_seq and last_ack_frame is not None:
                    uart.write(last_ack_frame)
                    print("[VL] duplicate command seq={}, ACK repeated".format(request_seq))
                    continue

                command = decode_command(received[2])
                status = STATUS_OK
                if command is None:
                    status = STATUS_BAD_PAYLOAD
                elif command["command_id"] == CMD_SET_MODE:
                    mode = command["mode"]
                elif command["command_id"] == CMD_START_STREAM:
                    stream_enabled = True
                elif command["command_id"] == CMD_STOP_STREAM:
                    stream_enabled = False
                elif command["command_id"] == CMD_SET_REFERENCE:
                    reference_x = command["arg0"]
                    reference_y = command["arg1"]
                else:
                    status = STATUS_UNSUPPORTED

                last_ack_frame = pack_ack(seq, request_seq, status)
                seq = (seq + 1) & 0xFF
                last_request_seq = request_seq
                uart.write(last_ack_frame)
                print("[VL] command seq={} id={} mode={} status={}".format(
                    request_seq,
                    -1 if command is None else command["command_id"],
                    mode,
                    status,
                ))

    if ticks_diff(now, last_print_ms) >= 1000:
        print("[VL] sending, seq={}".format(seq))
        last_print_ms = now

    sleep_ms(1)
