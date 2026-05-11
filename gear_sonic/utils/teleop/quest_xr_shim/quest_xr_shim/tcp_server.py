"""TCP/JSON listener for the Meta Quest Pro Unity teleop APK.

The APK opens a client connection and streams one JSON object per frame, e.g.::

    {
      "predictTime": 123955604.744,
      "appState": {"focus": true},
      "Head": {"pose": "x,y,z,qx,qy,qz,qw", "status": 3},
      "Controller": {
        "left":  {"pose": "...", "axisX": 0, "axisY": 0, "axisClick": false,
                   "grip": 0, "trigger": 0,
                   "primaryButton": false, "secondaryButton": false, "menuButton": false},
        "right": {... same shape ...}
      },
      "timeStampNs": 1768814265964327168,
      "Input": 2
    }

The server tolerates both newline-delimited and back-to-back JSON streams by
incrementally decoding the receive buffer.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from typing import Optional

from .coords import parse_pose_string

DEFAULT_PORT = 63901

# Wire format from the Quest Unity APK (xr_teleoperate-compatible framing):
#   [u16 LE magic 0x6d3f] [u32 LE payload_len] [payload_len B JSON envelope] [9 B trailer]
# The JSON envelope looks like {"functionName":"Tracking","value":"<stringified inner JSON>"};
# the actual tracking payload (Head, Controller, timeStampNs, ...) is the inner JSON
# inside ``value``.
_FRAME_MAGIC = b"\x3f\x6d"
_HEADER_SIZE = 6
_TRAILER_SIZE = 9


class QuestTCPServer:
    """Background TCP listener that exposes the latest Quest frame.

    Holds the most recent valid pose for the head and each controller so a
    momentary tracking dropout (sentinel ``0,0,0,0,0,0,-1``) does not zero the
    downstream pipeline.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = DEFAULT_PORT):
        self.host = host
        self.port = port
        self._latest: Optional[dict] = None
        self._prev_head: Optional[list[float]] = None
        self._prev_left: Optional[list[float]] = None
        self._prev_right: Optional[list[float]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="quest-xr-tcp")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def get_latest(self) -> Optional[dict]:
        with self._lock:
            return self._latest

    # ------------------------------------------------------------------ socket
    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(1)
        sock.settimeout(0.5)
        print(
            f"[quest_xr_shim] Listening for Quest client on tcp://{self.host}:{self.port}"
        )
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                print(f"[quest_xr_shim] Quest connected from {addr}")
                try:
                    self._handle_client(conn)
                finally:
                    try:
                        conn.close()
                    except OSError:
                        pass
                if not self._stop.is_set():
                    print("[quest_xr_shim] Quest disconnected; awaiting reconnect")
        finally:
            try:
                sock.close()
            except OSError:
                pass

    def _handle_client(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = conn.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while True:
                if len(buf) >= 2 and buf[:2] != _FRAME_MAGIC:
                    idx = buf.find(_FRAME_MAGIC, 1)
                    if idx < 0:
                        # Keep last byte in case magic spans the next chunk.
                        buf = buf[-1:]
                        break
                    buf = buf[idx:]
                if len(buf) < _HEADER_SIZE:
                    break
                payload_len = struct.unpack_from("<I", buf, 2)[0]
                frame_size = _HEADER_SIZE + payload_len + _TRAILER_SIZE
                if len(buf) < frame_size:
                    break
                payload = buf[_HEADER_SIZE : _HEADER_SIZE + payload_len]
                buf = buf[frame_size:]
                try:
                    envelope = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                if not isinstance(envelope, dict):
                    continue
                if envelope.get("functionName") != "Tracking":
                    continue
                try:
                    inner = json.loads(envelope["value"])
                except (json.JSONDecodeError, KeyError, TypeError):
                    continue
                if isinstance(inner, dict):
                    self._update(inner)

    # ------------------------------------------------------------------ state
    def _update(self, obj: dict) -> None:
        head_pose, head_ok = parse_pose_string(obj.get("Head", {}).get("pose", ""))
        if head_ok:
            self._prev_head = head_pose
        elif self._prev_head is not None:
            head_pose = self._prev_head

        ctrl = obj.get("Controller", {}) or {}
        left = ctrl.get("left", {}) or {}
        right = ctrl.get("right", {}) or {}

        l_pose, l_ok = parse_pose_string(left.get("pose", ""))
        if l_ok:
            self._prev_left = l_pose
        elif self._prev_left is not None:
            l_pose = self._prev_left

        r_pose, r_ok = parse_pose_string(right.get("pose", ""))
        if r_ok:
            self._prev_right = r_pose
        elif self._prev_right is not None:
            r_pose = self._prev_right

        sample = {
            "head_pose": head_pose,
            "left_pose": l_pose,
            "right_pose": r_pose,
            "left_axisX": float(left.get("axisX", 0.0)),
            "left_axisY": float(left.get("axisY", 0.0)),
            "right_axisX": float(right.get("axisX", 0.0)),
            "right_axisY": float(right.get("axisY", 0.0)),
            "left_axis_click": bool(left.get("axisClick", False)),
            "right_axis_click": bool(right.get("axisClick", False)),
            "left_grip": float(left.get("grip", 0.0)),
            "right_grip": float(right.get("grip", 0.0)),
            "left_trigger": float(left.get("trigger", 0.0)),
            "right_trigger": float(right.get("trigger", 0.0)),
            "left_primary": bool(left.get("primaryButton", False)),
            "left_secondary": bool(left.get("secondaryButton", False)),
            "right_primary": bool(right.get("primaryButton", False)),
            "right_secondary": bool(right.get("secondaryButton", False)),
            "left_menu": bool(left.get("menuButton", False)),
            "right_menu": bool(right.get("menuButton", False)),
            "timeStampNs": int(obj.get("timeStampNs", time.time_ns())),
        }
        with self._lock:
            self._latest = sample
