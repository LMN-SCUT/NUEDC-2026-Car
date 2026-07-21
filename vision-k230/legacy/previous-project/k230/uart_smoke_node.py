"""CanMV/K230 UART smoke sender with ACK/feedback print."""

import struct
import time

try:
    from machine import UART
except Exception:
    from Maix import UART

SOF1 = 0xAA
SOF2 = 0x55

TYPE_OBS = 0x01
TYPE_HEARTBEAT = 0x02
TYPE_FEEDBACK = 0x81
TYPE_ACK = 0x90
TYPE_PARAM = 0x10

UART_ID = 3
TX_PIN = 32
RX_PIN = 33
BAUD = 115200

ACK_STRUCT = struct.Struct("<BBhhhB")
FB_STRUCT = struct.Struct("<BHHHH")
PARAM_STRUCT = struct.Struct("<Bhhh")

SEND_SET_ZERO = True
SET_ZERO_DELAY_MS = 2000
SET_ZERO_RETRY_MS = 1000
SET_ZERO_MAX_RETRY = 3
RX_TIMEOUT_MS = 2000


def checksum(data):
    total = 0
    for b in data:
        total += b
    return total & 0xFF


def pack_frame(frame_type, seq, payload):
    out = bytearray()
    out.append(SOF1)
    out.append(SOF2)
    out.append(frame_type & 0xFF)
    out.append(seq & 0xFF)
    out.append(len(payload) & 0xFF)
    out.extend(payload)
    cs = checksum(out[2:])
    out.append(cs)
    return bytes(out)


def pack_obs(ts_ms, seq, center_u, center_v, err_u, err_v, confidence, target_lost, mode_hint):
    if target_lost:
        err_u = 0
        err_v = 0
        confidence = 0
    payload = struct.pack(
        "<IhhhhBBBB",
        ts_ms & 0xFFFFFFFF,
        int(center_u),
        int(center_v),
        int(err_u),
        int(err_v),
        int(confidence) & 0xFF,
        1 if target_lost else 0,
        int(mode_hint) & 0xFF,
        0,
    )
    return pack_frame(TYPE_OBS, seq, payload)


def pack_hb(ts_ms, seq, status_bits):
    payload = struct.pack("<IH", ts_ms & 0xFFFFFFFF, status_bits & 0xFFFF)
    return pack_frame(TYPE_HEARTBEAT, seq, payload)


def pack_param(seq, cmd_id, arg0, arg1, arg2):
    payload = PARAM_STRUCT.pack(cmd_id & 0xFF, int(arg0), int(arg1), int(arg2))
    return pack_frame(TYPE_PARAM, seq, payload)


def parser_init():
    return {
        "state": 0,
        "frame_type": 0,
        "seq": 0,
        "length": 0,
        "payload": bytearray(),
        "sum": 0,
    }


def parser_reset(p):
    p["state"] = 0
    p["frame_type"] = 0
    p["seq"] = 0
    p["length"] = 0
    p["payload"] = bytearray()
    p["sum"] = 0


def parser_push(p, byte):
    b = byte & 0xFF

    if p["state"] == 0:
        if b == SOF1:
            p["state"] = 1
        return None

    if p["state"] == 1:
        if b == SOF2:
            p["state"] = 2
            p["sum"] = 0
        elif b != SOF1:
            p["state"] = 0
        return None

    if p["state"] == 2:
        p["frame_type"] = b
        p["sum"] = b
        p["state"] = 3
        return None

    if p["state"] == 3:
        p["seq"] = b
        p["sum"] = (p["sum"] + b) & 0xFF
        p["state"] = 4
        return None

    if p["state"] == 4:
        p["length"] = b
        p["sum"] = (p["sum"] + b) & 0xFF
        p["payload"] = bytearray()
        if p["length"] == 0:
            p["state"] = 6
        else:
            p["state"] = 5
        return None

    if p["state"] == 5:
        p["payload"].append(b)
        p["sum"] = (p["sum"] + b) & 0xFF
        if len(p["payload"]) >= p["length"]:
            p["state"] = 6
        return None

    if p["state"] == 6:
        ok = (p["sum"] & 0xFF) == b
        if ok:
            frame = (p["frame_type"], p["seq"], bytes(p["payload"]))
        else:
            frame = None
        parser_reset(p)
        return frame

    parser_reset(p)
    return None


def read_uart_bytes(uart_obj):
    try:
        n = uart_obj.any()
    except Exception:
        n = 0
    if n and n > 0:
        return uart_obj.read(n)
    return None


def handle_frame(frame_type, payload):
    global ack_zero, last_rx_ms, last_ack_ms, last_fb_ms, pending_zero
    if frame_type == TYPE_ACK and len(payload) == ACK_STRUCT.size:
        ack_code, cmd_id, arg0, arg1, arg2, _ = ACK_STRUCT.unpack(payload)
        print("[ACK] code={} cmd=0x{:02X} a0={} a1={} a2={}".format(
            ack_code, cmd_id, arg0, arg1, arg2
        ))
        last_ack_ms = time.ticks_ms()
        if cmd_id == 0x03 and ack_code == 0:
            ack_zero = True
            pending_zero = False
        return

    if frame_type == TYPE_FEEDBACK and len(payload) == FB_STRUCT.size:
        ctrl_state, yaw_pwm, pitch_pwm, fault_code, lap_phase_q15 = FB_STRUCT.unpack(payload)
        print("[FB] st={} yaw={} pit={} fault=0x{:04X} phase={}".format(
            ctrl_state, yaw_pwm, pitch_pwm, fault_code, lap_phase_q15
        ))
        last_fb_ms = time.ticks_ms()
        return

    print("[RX] type=0x{:02X} len={}".format(frame_type, len(payload)))


try:
    uart = UART(UART_ID, BAUD, tx=TX_PIN, rx=RX_PIN)
except Exception:
    uart = UART(UART_ID, BAUD)

last_obs_ms = time.ticks_ms()
last_hb_ms = time.ticks_ms()
parser = parser_init()
seq = 0
last_print_ms = time.ticks_ms()
ack_zero = False
last_zero_ms = time.ticks_ms()
zero_retry = 0
pending_zero = False
last_cmd_ms = time.ticks_ms()
last_rx_ms = time.ticks_ms()
last_ack_ms = 0
last_fb_ms = 0
last_timeout_ms = time.ticks_ms()

while True:
    now = time.ticks_ms()

    if time.ticks_diff(now, last_obs_ms) >= 33:
        pkt = pack_obs(now, seq, 320, 240, 10, -5, 90, 0, 0)
        uart.write(pkt)
        seq = (seq + 1) & 0xFF
        last_obs_ms = now

    if time.ticks_diff(now, last_hb_ms) >= 100:
        pkt = pack_hb(now, seq, 0x0007)
        uart.write(pkt)
        seq = (seq + 1) & 0xFF
        last_hb_ms = now

    data = read_uart_bytes(uart)
    if data:
        for b in data:
            frame = parser_push(parser, b)
            if frame:
                last_rx_ms = time.ticks_ms()
                handle_frame(frame[0], frame[2])

    if SEND_SET_ZERO and not ack_zero:
        if time.ticks_diff(now, last_zero_ms) >= SET_ZERO_DELAY_MS:
            if zero_retry < SET_ZERO_MAX_RETRY:
                pkt = pack_param(seq, 0x03, 0, 0, 0)
                uart.write(pkt)
                seq = (seq + 1) & 0xFF
                zero_retry += 1
                last_zero_ms = now + SET_ZERO_RETRY_MS
                pending_zero = True
                last_cmd_ms = now
                print("[CMD] SET_ZERO send, retry={}".format(zero_retry))

    if time.ticks_diff(now, last_timeout_ms) >= 1000:
        if time.ticks_diff(now, last_rx_ms) >= RX_TIMEOUT_MS:
            print("[WARN] RX timeout {}ms".format(time.ticks_diff(now, last_rx_ms)))
        elif last_fb_ms and time.ticks_diff(now, last_fb_ms) >= RX_TIMEOUT_MS:
            print("[WARN] FB timeout {}ms".format(time.ticks_diff(now, last_fb_ms)))
        elif pending_zero and time.ticks_diff(now, last_cmd_ms) >= RX_TIMEOUT_MS:
            print("[WARN] ACK timeout {}ms".format(time.ticks_diff(now, last_cmd_ms)))
            pending_zero = False
        last_timeout_ms = now

    if time.ticks_diff(now, last_print_ms) >= 1000:
        print("[TX] obs/hb running, seq={}".format(seq))
        last_print_ms = now

    time.sleep_ms(1)

