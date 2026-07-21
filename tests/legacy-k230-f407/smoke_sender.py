from __future__ import annotations

import argparse
import time

import serial

from protocol import HeartbeatPayload, ObsPayload, pack_heartbeat, pack_obs


def now_ms() -> int:
    return int(time.monotonic() * 1000)


def main() -> None:
    ap = argparse.ArgumentParser(description="Simple smoke sender for K230<->F407 protocol")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=200)
    args = ap.parse_args()

    seq = 0
    with serial.Serial(args.port, args.baud, timeout=0.01) as ser:
        for i in range(args.count):
            ts = now_ms()
            obs = ObsPayload(
                timestamp_ms=ts,
                center_u=320,
                center_v=240,
                err_u=(i % 21) - 10,
                err_v=((i * 2) % 21) - 10,
                confidence=90,
                target_lost=0,
            )
            ser.write(pack_obs(seq, obs))
            seq = (seq + 1) & 0xFF

            if i % 5 == 0:
                hb = HeartbeatPayload(timestamp_ms=ts, status_bits=0x0007)
                ser.write(pack_heartbeat(seq, hb))
                seq = (seq + 1) & 0xFF

            time.sleep(0.02)

    print("smoke_sender done")


if __name__ == "__main__":
    main()

