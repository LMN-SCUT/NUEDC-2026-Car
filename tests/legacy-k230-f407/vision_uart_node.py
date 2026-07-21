from __future__ import annotations

import argparse
import math
import time

import serial

from protocol import (
    FrameType,
    HeartbeatPayload,
    ObsPayload,
    StreamParser,
    decode_ack,
    decode_feedback,
    pack_heartbeat,
    pack_obs,
)


def now_ms() -> int:
    return int(time.monotonic() * 1000)


def build_demo_obs(ts: int, t_sec: float, lost: bool) -> ObsPayload:
    center_u = int(320 + 40 * math.sin(t_sec * 1.2))
    center_v = int(240 + 30 * math.cos(t_sec * 0.8))
    err_u = int(15 * math.sin(t_sec * 1.6))
    err_v = int(10 * math.cos(t_sec * 1.1))
    conf = 85 if not lost else 0
    return ObsPayload(
        timestamp_ms=ts,
        center_u=center_u,
        center_v=center_v,
        err_u=err_u,
        err_v=err_v,
        confidence=conf,
        target_lost=1 if lost else 0,
        mode_hint=0,
        reserved=0,
    )


def process_rx(parser: StreamParser, ser: serial.Serial) -> None:
    data = ser.read(ser.in_waiting or 1)
    for b in data:
        frame = parser.push(b)
        if frame is None:
            continue

        if frame.frame_type == FrameType.FEEDBACK:
            info = decode_feedback(frame.payload)
            if info is not None:
                print(
                    f"[FB] st={info['ctrl_state']} yaw={info['yaw_pwm']} "
                    f"pit={info['pitch_pwm']} fault=0x{info['fault_code']:04X} "
                    f"phase={info['lap_phase_q15']}"
                )
        elif frame.frame_type == FrameType.ACK:
            info = decode_ack(frame.payload)
            if info is not None:
                print(f"[ACK] code={info['ack_code']} raw={info['raw'].hex()}")
        else:
            print(f"[RX] type=0x{frame.frame_type:02X} len={len(frame.payload)}")


def main() -> None:
    ap = argparse.ArgumentParser(description="K230 vision UART node demo")
    ap.add_argument("--port", required=True, help="UART port, e.g. COM9 or /dev/ttyS1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--fps", type=float, default=30.0, help="OBS send rate")
    ap.add_argument("--hb-ms", type=int, default=100, help="heartbeat period")
    ap.add_argument("--lost-every", type=float, default=0.0, help="inject lost window every N seconds")
    ap.add_argument("--run-sec", type=float, default=0.0, help="0 means run forever")
    args = ap.parse_args()

    if args.fps <= 0:
        raise ValueError("fps must be > 0")

    ser = serial.Serial(args.port, args.baud, timeout=0.002)
    parser = StreamParser()

    seq = 0
    obs_period = 1.0 / args.fps
    next_obs = time.monotonic()
    next_hb = time.monotonic()
    t0 = time.monotonic()

    print(f"[INFO] open {args.port} @ {args.baud}")
    print("[INFO] sending OBS + HEARTBEAT frames")

    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0

            if args.run_sec > 0 and elapsed >= args.run_sec:
                print("[INFO] run finished")
                break

            if now >= next_obs:
                ts = now_ms()
                lost = False
                if args.lost_every > 1e-6:
                    phase = elapsed % args.lost_every
                    lost = phase < 0.5

                obs = build_demo_obs(ts, elapsed, lost)
                pkt = pack_obs(seq, obs)
                ser.write(pkt)
                seq = (seq + 1) & 0xFF
                next_obs += obs_period

            if now >= next_hb:
                hb = HeartbeatPayload(timestamp_ms=now_ms(), status_bits=0x0007)
                pkt = pack_heartbeat(seq, hb)
                ser.write(pkt)
                seq = (seq + 1) & 0xFF
                next_hb += args.hb_ms / 1000.0

            process_rx(parser, ser)
            time.sleep(0.001)

    finally:
        ser.close()
        print("[INFO] closed")


if __name__ == "__main__":
    main()

