"""Subscribe to the manager's `planner` topic and print mode/movement/facing/speed.

Use to verify §4 step 3 (left stick -> movement) and step 4 (right stick -> facing/yaw)
without a simulator. Run alongside `pico_manager_thread_server.py`.

Wire format: see gear_sonic/utils/teleop/zmq/zmq_planner_sender.py
  [b"planner"][1280-byte JSON header][packed binary fields]
"""

import argparse
import json
import struct
import sys
import time

import zmq

HEADER_SIZE = 1280
DTYPE_FMT = {"f32": ("<f", 4), "f64": ("<d", 8), "i32": ("<i", 4), "i64": ("<q", 8), "u8": ("<B", 1)}


def decode_planner(raw: bytes) -> dict:
    if not raw.startswith(b"planner"):
        raise ValueError(f"unexpected topic prefix: {raw[:16]!r}")
    off = len(b"planner")
    header_bytes = raw[off : off + HEADER_SIZE].split(b"\x00", 1)[0]
    header = json.loads(header_bytes.decode("utf-8"))
    off += HEADER_SIZE
    out: dict = {}
    for field in header["fields"]:
        name, dtype, shape = field["name"], field["dtype"], field["shape"]
        if dtype not in DTYPE_FMT:
            return out
        fmt, itemsize = DTYPE_FMT[dtype]
        n = 1
        for s in shape:
            n *= s
        vals = struct.unpack("<" + fmt[1] * n, raw[off : off + n * itemsize])
        out[name] = vals[0] if n == 1 else list(vals)
        off += n * itemsize
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument(
        "--decimate",
        type=int,
        default=10,
        help="print every Nth frame; manager streams at ~50 Hz (default: 10 -> ~5 Hz)",
    )
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{args.host}:{args.port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "planner")
    sock.setsockopt(zmq.RCVTIMEO, 2000)
    print(f"[debug_planner_sub] connected tcp://{args.host}:{args.port}, topic=planner")
    print("[debug_planner_sub] push left stick -> mv should change; right stick -> face should rotate")

    frame = 0
    last_mode = None
    last_print = time.monotonic()
    while True:
        try:
            raw = sock.recv()
        except zmq.Again:
            print("\n[debug_planner_sub] no message in 2s -- is the manager running on that port?")
            continue
        except KeyboardInterrupt:
            print("\n[debug_planner_sub] bye")
            return

        try:
            msg = decode_planner(raw)
        except Exception as e:
            print(f"\n[debug_planner_sub] decode error: {e}")
            continue

        mode = msg.get("mode")
        mv = msg.get("movement", [0.0, 0.0, 0.0])
        face = msg.get("facing", [0.0, 0.0, 0.0])
        speed = msg.get("speed", 0.0)

        if mode != last_mode:
            now = time.monotonic()
            sys.stdout.write(f"\n[mode] {last_mode} -> {mode} (after {now - last_print:.1f}s)\n")
            last_mode = mode

        frame += 1
        if frame % args.decimate == 0:
            sys.stdout.write(
                f"\rmode={mode} mv=({mv[0]:+.3f},{mv[1]:+.3f},{mv[2]:+.3f}) "
                f"face=({face[0]:+.3f},{face[1]:+.3f},{face[2]:+.3f}) spd={speed:+.3f}   "
            )
            sys.stdout.flush()
            last_print = time.monotonic()


if __name__ == "__main__":
    main()
