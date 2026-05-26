"""Autopilot daemon for SONIC teleop data collection.

A single long-running process that does both:
  * record  — driven by Y/X tap (or right_grip+A / right_grip+B) edges from the
              manager. Subscribes to ``planner`` and writes
              walk_to_table.json / return_to_start.json.
  * replay  — driven by stick-click edges from the manager. Publishes
              ``autopilot_cmd`` (and optionally ``reset_cmd`` after return).

The operator never talks to this process directly: only the manager reads the
headset and emits topic edges. Record and replay are mutually exclusive states
in a single state machine.

Topics:
  IN  (SUB tcp://localhost:5556)  ``planner``           — frames captured while recording
  IN  (SUB tcp://localhost:5556)  ``autopilot_request`` — replay forward/return/cancel
  IN  (SUB tcp://localhost:5556)  ``record_request``    — record forward/return toggle
  OUT (PUB tcp://*:5558)          ``autopilot_cmd``     — playback frames
  OUT (PUB tcp://*:5558)          ``reset_cmd``         — emitted after return playback

``autopilot_cmd`` fields (pack_pose_message):
  active:    bool[1]   1 while a trajectory is playing
  kind:      i32[1]    1=forward 2=return 0=idle
  mode:      i32[1]    LocomotionMode value
  movement:  f32[3]
  facing:    f32[3]
  speed:     f32[1]

``reset_cmd`` fields:
  trigger:   bool[1]   one-shot reset request (sim_env.reset + jug to spawn pos)

``autopilot_request`` fields (set by manager):
  action:    i32[1]    0=none 1=forward_toggle 2=return_toggle 3=cancel

``record_request`` fields (set by manager):
  action:    i32[1]    0=none 1=forward_toggle 2=return_toggle

Usage:
    python gear_sonic/scripts/auto_pilot.py --play
"""

from dataclasses import dataclass
import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import zmq

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import pack_pose_message


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

# Manager (publisher of planner/autopilot_request) — autopilot subscribes here.
DEFAULT_MANAGER_HOST = "localhost"
DEFAULT_MANAGER_PORT = 5556

# Autopilot publisher — manager and sim subscribe here.
DEFAULT_AUTOPILOT_PORT = 5558

# Record/play loop rate. Matches PlannerStreamer.poll_hz default (20 Hz).
DEFAULT_HZ = 20

# Default traj directory (relative to repo root).
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_TRAJ_DIR = _REPO_ROOT / "gear_sonic" / "data" / "autopilot_trajs"

TRAJ_FILES = {
    "forward": "walk_to_table.json",
    "return": "return_to_start.json",
}

# Match LocomotionMode enum from pico_manager_thread_server.py.
_LOCOMOTION_IDLE = 0

# Match autopilot_request action codes published by the manager (replay).
ACTION_NONE = 0
ACTION_FORWARD_TOGGLE = 1
ACTION_RETURN_TOGGLE = 2
ACTION_CANCEL = 3

# record_request action codes published by the manager (record).
REC_NONE = 0
REC_FORWARD_TOGGLE = 1
REC_RETURN_TOGGLE = 2


# ---------------------------------------------------------------------------
# Header unpacking (mirror of unpack_pose_message in run_data_exporter.py)
# ---------------------------------------------------------------------------

_HEADER_SIZE = 1280
_DTYPE_MAP = {
    "f32": np.float32,
    "f64": np.float64,
    "i32": np.int32,
    "i64": np.int64,
    "bool": np.bool_,
}


def _unpack(packed: bytes, topic: str) -> dict:
    topic_bytes = topic.encode("utf-8")
    if not packed.startswith(topic_bytes):
        raise ValueError(f"topic mismatch: expected {topic!r}")
    offset = len(topic_bytes)
    header_blob = packed[offset : offset + _HEADER_SIZE]
    null = header_blob.find(b"\x00")
    if null >= 0:
        header_blob = header_blob[:null]
    header = json.loads(header_blob.decode("utf-8"))
    out = {}
    cursor = offset + _HEADER_SIZE
    for field in header.get("fields", []):
        dtype = _DTYPE_MAP.get(field["dtype"], np.float32)
        shape = tuple(field["shape"])
        nbytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        out[field["name"]] = (
            np.frombuffer(packed[cursor : cursor + nbytes], dtype=dtype).reshape(shape).copy()
        )
        cursor += nbytes
    return out


# ---------------------------------------------------------------------------
# Trajectory I/O
# ---------------------------------------------------------------------------


@dataclass
class TrajFrame:
    t: float  # seconds from start of recording
    mode: int
    movement: list  # [3]
    facing: list  # [3]
    speed: float


def _save_traj(path: Path, dt: float, frames: list[TrajFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "dt": dt,
        "frames": [
            {
                "t": float(f.t),
                "mode": int(f.mode),
                "movement": [float(x) for x in f.movement],
                "facing": [float(x) for x in f.facing],
                "speed": float(f.speed),
            }
            for f in frames
        ],
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
    print(f"[Autopilot] Saved {len(frames)} frames ({frames[-1].t:.2f}s) → {path}")


def _load_traj(path: Path) -> tuple[float, list[TrajFrame]]:
    with open(path, "r", encoding="utf-8") as fh:
        payload = json.load(fh)
    frames = [
        TrajFrame(
            t=float(f["t"]),
            mode=int(f["mode"]),
            movement=list(f["movement"]),
            facing=list(f["facing"]),
            speed=float(f["speed"]),
        )
        for f in payload["frames"]
    ]
    return float(payload.get("dt", 1.0 / DEFAULT_HZ)), frames


# ---------------------------------------------------------------------------
# Planner-frame parsing (used while recording)
# ---------------------------------------------------------------------------


def _planner_to_frame(data: dict, t: float) -> TrajFrame:
    mode_v = int(data["mode"].flat[0]) if "mode" in data else 0
    movement = (
        data["movement"].flatten().astype(np.float32).tolist()
        if "movement" in data and data["movement"].size == 3
        else [0.0, 0.0, 0.0]
    )
    facing = (
        data["facing"].flatten().astype(np.float32).tolist()
        if "facing" in data and data["facing"].size == 3
        else [1.0, 0.0, 0.0]
    )
    speed_v = float(data["speed"].flat[0]) if "speed" in data else -1.0
    return TrajFrame(t=t, mode=mode_v, movement=movement, facing=facing, speed=speed_v)


# ---------------------------------------------------------------------------
# Server: replay + record state machine
# ---------------------------------------------------------------------------


def _build_autopilot_cmd(active: bool, kind_code: int, frame: Optional[TrajFrame]) -> bytes:
    if frame is None:
        movement = np.zeros(3, dtype=np.float32)
        facing = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        speed = -1.0
        mode_v = _LOCOMOTION_IDLE
    else:
        movement = np.array(frame.movement, dtype=np.float32)
        facing = np.array(frame.facing, dtype=np.float32)
        speed = float(frame.speed)
        mode_v = int(frame.mode)
    return pack_pose_message(
        {
            "active": np.array([active], dtype=np.bool_),
            "kind": np.array([kind_code], dtype=np.int32),
            "mode": np.array([mode_v], dtype=np.int32),
            "movement": movement,
            "facing": facing,
            "speed": np.array([speed], dtype=np.float32),
        },
        topic="autopilot_cmd",
    )


def _build_reset_cmd() -> bytes:
    return pack_pose_message(
        {"trigger": np.array([True], dtype=np.bool_)},
        topic="reset_cmd",
    )


def _play(
    manager_host: str,
    manager_port: int,
    publish_port: int,
    traj_dir: Path,
    sim_reset: bool,
) -> None:
    """Unified replay+record daemon.

    States (mutually exclusive):
      idle | play_forward | play_return | record_forward | record_return

    Replay edges come on ``autopilot_request`` (stick clicks from manager).
    Record edges come on ``record_request`` (right_grip+A / right_grip+B taps).
    While recording, planner frames are appended to a buffer; the buffer is
    flushed to JSON on the stop toggle.
    """

    traj_dir.mkdir(parents=True, exist_ok=True)
    cached: dict[str, tuple[float, list[TrajFrame]]] = {}

    # Persistent trace file: every state transition / save / important event
    # gets appended here with a timestamp, so the operator can grep this file
    # afterwards even if tmux scrollback flushed.
    trace_path = traj_dir / ".daemon_trace.log"
    trace_fh = open(trace_path, "a", encoding="utf-8", buffering=1)  # line-buffered
    trace_fh.write(f"\n=== daemon start {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    def _trace(msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        try:
            trace_fh.write(line + "\n")
        except Exception:
            pass

    def _banner(msg: str) -> None:
        bar = "=" * 60
        _trace(f"\n{bar}\n{msg}\n{bar}")

    def _ensure(kind: str) -> Optional[tuple[float, list[TrajFrame]]]:
        if kind in cached:
            return cached[kind]
        path = traj_dir / TRAJ_FILES[kind]
        if not path.exists():
            print(f"[Autopilot] WARN: trajectory not found: {path}")
            return None
        cached[kind] = _load_traj(path)
        print(f"[Autopilot] loaded {kind}: {len(cached[kind][1])} frames from {path}")
        return cached[kind]

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{manager_host}:{manager_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"autopilot_request")
    sub.setsockopt(zmq.SUBSCRIBE, b"record_request")
    sub.setsockopt(zmq.SUBSCRIBE, b"planner")
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{publish_port}")
    # Give SUBs a beat to attach before we start blasting heartbeats.
    time.sleep(0.2)

    _trace(
        f"[Autopilot] daemon up | SUB tcp://{manager_host}:{manager_port} "
        f"(autopilot_request,record_request,planner) "
        f"| PUB tcp://*:{publish_port}/autopilot_cmd"
    )
    _trace(f"[Autopilot] trace file: {trace_path}")

    # Possible states: 'idle' | 'play_forward' | 'play_return' |
    #                  'record_forward' | 'record_return'
    state = "idle"
    play_start: Optional[float] = None
    frame_idx = 0
    rec_t0: Optional[float] = None
    rec_frames: list[TrajFrame] = []

    interrupted = {"flag": False}

    def _handle_sigint(_signum, _frame):
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)

    dt = 1.0 / DEFAULT_HZ
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    next_tick = time.monotonic()
    last_heartbeat = time.monotonic()

    def _engage_play(kind: str) -> bool:
        nonlocal state, play_start, frame_idx
        clip = _ensure(kind)
        if clip is None:
            return False
        state = f"play_{kind}"
        play_start = time.monotonic()
        frame_idx = 0
        _banner(f"[Autopilot] >>> PLAY ENGAGE {kind} ({len(clip[1])} frames) <<<")
        return True

    def _disengage_play(reason: str) -> None:
        nonlocal state, play_start, frame_idx
        if state.startswith("play_"):
            _banner(f"[Autopilot] >>> PLAY DISENGAGE {state} ({reason}) <<<")
        state = "idle"
        play_start = None
        frame_idx = 0

    def _engage_record(kind: str) -> None:
        nonlocal state, rec_t0, rec_frames
        state = f"record_{kind}"
        rec_t0 = time.monotonic()
        rec_frames = []
        _banner(f"[Autopilot] >>> RECORD ENGAGE {kind} — t0=now, buffer reset <<<")

    def _finish_record(kind: str) -> None:
        nonlocal state, rec_t0, rec_frames
        out_path = traj_dir / TRAJ_FILES[kind]
        if not rec_frames:
            _banner(f"[Autopilot] >>> RECORD STOP {kind} — EMPTY buffer, nothing saved <<<")
        else:
            dt_est = rec_frames[-1].t / max(1, len(rec_frames) - 1)
            _save_traj(out_path, dt_est, rec_frames)
            _banner(
                f"[Autopilot] >>> SAVED {out_path.name}: {len(rec_frames)} frames "
                f"({rec_frames[-1].t:.2f}s) <<<"
            )
        state = "idle"
        rec_t0 = None
        rec_frames = []

    try:
        while not interrupted["flag"]:
            # Drain all pending messages this tick.
            events = dict(poller.poll(timeout=0))
            while sub in events:
                raw = sub.recv(zmq.NOBLOCK)
                # Topic is the prefix up to the first byte that doesn't match
                # one of our known topics — easiest is to try each.
                if raw.startswith(b"autopilot_request"):
                    try:
                        msg = _unpack(raw, "autopilot_request")
                    except Exception:
                        msg = None
                    if msg is not None:
                        action = int(msg["action"].flat[0]) if "action" in msg else 0
                        if state.startswith("record_"):
                            # Interlocked: ignore replay edges while recording.
                            if action != ACTION_NONE:
                                print(
                                    f"[Autopilot] replay action {action} ignored — recording in progress"
                                )
                        elif action == ACTION_FORWARD_TOGGLE:
                            if state == "play_forward":
                                _disengage_play("forward toggle off")
                            elif state == "idle":
                                _engage_play("forward")
                        elif action == ACTION_RETURN_TOGGLE:
                            if state == "play_return":
                                _disengage_play("return toggle off")
                            elif state == "idle":
                                _engage_play("return")
                        elif action == ACTION_CANCEL:
                            _disengage_play("cancel")
                elif raw.startswith(b"record_request"):
                    try:
                        msg = _unpack(raw, "record_request")
                    except Exception:
                        msg = None
                    if msg is not None:
                        action = int(msg["action"].flat[0]) if "action" in msg else 0
                        if state.startswith("play_"):
                            if action != REC_NONE:
                                print(
                                    f"[Autopilot] record action {action} ignored — replay in progress"
                                )
                        elif action == REC_FORWARD_TOGGLE:
                            if state == "record_forward":
                                _finish_record("forward")
                            elif state == "idle":
                                _engage_record("forward")
                            else:  # record_return
                                print("[Autopilot] forward record toggle ignored — return record in progress")
                        elif action == REC_RETURN_TOGGLE:
                            if state == "record_return":
                                _finish_record("return")
                            elif state == "idle":
                                _engage_record("return")
                            else:
                                print("[Autopilot] return record toggle ignored — forward record in progress")
                elif raw.startswith(b"planner"):
                    if state.startswith("record_") and rec_t0 is not None:
                        try:
                            data = _unpack(raw, "planner")
                        except Exception:
                            data = None
                        if data is not None:
                            t = time.monotonic() - rec_t0
                            rec_frames.append(_planner_to_frame(data, t))
                            if len(rec_frames) % (DEFAULT_HZ * 2) == 0:
                                kind = state.split("_", 1)[1]
                                _trace(
                                    f"[Autopilot] recording {kind}: {len(rec_frames):4d} frames ({t:5.2f}s)"
                                )
                events = dict(poller.poll(timeout=0))

            # Advance playback.
            if state in ("play_forward", "play_return"):
                kind = state.split("_", 1)[1]
                clip = cached.get(kind)
                if clip is None:
                    _disengage_play("missing clip")
                    continue
                _, frames = clip
                if kind == "forward":
                    # Forward is a hold-one-frame command, not a timed
                    # playback: walk_to_table.json's frames are all
                    # identical (constant forward command), and the
                    # operator decides when the robot has arrived by
                    # toggling forward off. We republish frames[0] every
                    # tick — right-stick yaw corrections in the manager
                    # steer it without time pressure.
                    pub.send(_build_autopilot_cmd(True, 1, frames[0]))
                else:
                    elapsed = time.monotonic() - play_start
                    while frame_idx + 1 < len(frames) and frames[frame_idx + 1].t <= elapsed:
                        frame_idx += 1
                    if elapsed >= frames[-1].t:
                        pub.send(_build_autopilot_cmd(True, 2, frames[-1]))
                        _disengage_play("clip finished (return)")
                        pub.send(_build_autopilot_cmd(False, 0, None))
                        if sim_reset:
                            pub.send(_build_reset_cmd())
                            print("[Autopilot] sent reset_cmd")
                    else:
                        pub.send(_build_autopilot_cmd(True, 2, frames[frame_idx]))
            else:
                # idle or recording → publish idle heartbeat so the manager
                # planner-side sees active=False and never tries to override.
                pub.send(_build_autopilot_cmd(False, 0, None))

            # Heartbeat every 3s so operator can always confirm current state
            # even if tmux scrollback flushes other prints.
            now_t = time.monotonic()
            if now_t - last_heartbeat > 3.0:
                if state.startswith("record_"):
                    extra = f" buf={len(rec_frames)} t={now_t - (rec_t0 or now_t):.1f}s"
                elif state.startswith("play_"):
                    extra = f" frame={frame_idx}"
                else:
                    extra = ""
                _trace(f"[Autopilot] HEARTBEAT state={state}{extra}")
                last_heartbeat = now_t

            # Pace.
            next_tick += dt
            sleep_t = next_tick - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_tick = time.monotonic()
    finally:
        # If we got Ctrl+C mid-record, flush what we have so the user doesn't
        # lose the buffer to a stray interrupt.
        if state.startswith("record_") and rec_frames:
            kind = state.split("_", 1)[1]
            print(f"[Autopilot] interrupted mid-record ({kind}) — flushing buffer")
            try:
                _finish_record(kind)
            except Exception as e:
                print(f"[Autopilot] flush failed: {e}")
        try:
            pub.send(_build_autopilot_cmd(False, 0, None))
        except Exception:
            pass
        sub.close()
        pub.close()
        try:
            trace_fh.write(f"=== daemon stop {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            trace_fh.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--play",
        action="store_true",
        default=True,
        help="(default) Run the unified record+replay daemon.",
    )
    p.add_argument(
        "--manager-host",
        type=str,
        default=DEFAULT_MANAGER_HOST,
        help=f"Manager ZMQ host (default: {DEFAULT_MANAGER_HOST})",
    )
    p.add_argument(
        "--manager-port",
        type=int,
        default=DEFAULT_MANAGER_PORT,
        help=f"Manager ZMQ port (default: {DEFAULT_MANAGER_PORT})",
    )
    p.add_argument(
        "--publish-port",
        type=int,
        default=DEFAULT_AUTOPILOT_PORT,
        help=f"Autopilot PUB port (default: {DEFAULT_AUTOPILOT_PORT})",
    )
    p.add_argument(
        "--traj-dir",
        type=str,
        default=str(DEFAULT_TRAJ_DIR),
        help=f"Trajectory directory (default: {DEFAULT_TRAJ_DIR})",
    )
    p.add_argument(
        "--no-sim-reset",
        dest="sim_reset",
        action="store_false",
        default=True,
        help="Disable the reset_cmd emitted after a return playback finishes.",
    )
    return p


def main(argv: Optional[list] = None) -> int:
    args = _build_parser().parse_args(argv)
    traj_dir = Path(args.traj_dir)
    _play(args.manager_host, args.manager_port, args.publish_port, traj_dir, args.sim_reset)
    return 0


if __name__ == "__main__":
    sys.exit(main())
