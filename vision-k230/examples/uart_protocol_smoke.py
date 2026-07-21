"""K230 protocol-v1 UART smoke sender.

Copy this file and shared/protocol/python/vision_link.py into the same K230
directory. It sends synthetic observations at 30 Hz and heartbeat at 2 Hz.
"""

import math
import time

from vision_link import (
    FLAG_RESULT_STABLE,
    FLAG_TARGET_VALID,
    STATUS_OK,
    STATUS_UNSUPPORTED,
    StreamParser,
    TYPE_COMMAND,
    decode_command,
    pack_ack,
    pack_heartbeat,
    pack_observation,
)

try:
    from machine import UART
except Exception:
    from Maix import UART

UART_ID = 3
TX_PIN = 32
RX_PIN = 33
BAUD = 115200

CMD_SET_MODE = 0x01
CMD_START_STREAM = 0x02
CMD_STOP_STREAM = 0x03
CMD_SET_REFERENCE = 0x04


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


def init_uart():
    try:
        return UART(UART_ID, BAUD, tx=TX_PIN, rx=RX_PIN)
    except Exception:
        return UART(UART_ID, BAUD)


uart = init_uart()
parser = StreamParser()
tx_seq = 0
stream_enabled = True
mode = 0
reference_x = 320
reference_y = 240
last_observation_ms = ticks_ms()
last_heartbeat_ms = ticks_ms()

print("[VL] smoke sender started: UART{} {} baud".format(UART_ID, BAUD))

while True:
    now = ticks_ms()

    if stream_enabled and ticks_diff(now, last_observation_ms) >= 33:
        phase = now / 1000.0
        center_x = reference_x + int(40 * math.sin(phase))
        center_y = reference_y + int(30 * math.cos(phase * 0.8))
        frame = pack_observation(
            tx_seq,
            now,
            center_x,
            center_y,
            center_x - reference_x,
            center_y - reference_y,
            90,
            FLAG_TARGET_VALID | FLAG_RESULT_STABLE,
            0,
            mode,
        )
        uart.write(frame)
        tx_seq = (tx_seq + 1) & 0xFF
        last_observation_ms = now

    if ticks_diff(now, last_heartbeat_ms) >= 500:
        frame = pack_heartbeat(tx_seq, now, 0x0003, 300)
        uart.write(frame)
        tx_seq = (tx_seq + 1) & 0xFF
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
                if received is None or received[1] != TYPE_COMMAND:
                    continue
                command = decode_command(received[3])
                status = STATUS_OK
                if command is None:
                    status = STATUS_UNSUPPORTED
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
                uart.write(pack_ack(tx_seq, received[1], received[2], status, 0))
                tx_seq = (tx_seq + 1) & 0xFF

    sleep_ms(1)
