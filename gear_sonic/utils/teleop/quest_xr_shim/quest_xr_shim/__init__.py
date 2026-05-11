"""``xrobotoolkit_sdk``-compatible facade backed by a Meta Quest Pro APK.

A single TCP/JSON server is started by :func:`init` and shared by all module-
level getters. The 17 ``xrt.*`` calls used by
``gear_sonic/scripts/pico_manager_thread_server.py`` are all covered here.

Switching backends:

.. code-block:: python

    # Real PICO 4 + foot trackers
    XR_BACKEND=pico python pico_manager_thread_server.py --manager ...

    # Meta Quest Pro + 2 controllers (default)
    python pico_manager_thread_server.py --manager --force-vr-3pt ...
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

from .smpl_fake import build_24x7_from_quest
from .tcp_server import DEFAULT_PORT, QuestTCPServer

__all__ = [
    "init",
    "is_body_data_available",
    "get_time_stamp_ns",
    "get_body_joints_pose",
    "get_left_trigger",
    "get_right_trigger",
    "get_left_grip",
    "get_right_grip",
    "get_A_button",
    "get_B_button",
    "get_X_button",
    "get_Y_button",
    "get_left_menu_button",
    "get_right_menu_button",
    "get_left_axis",
    "get_right_axis",
    "get_left_axis_click",
    "get_right_axis_click",
]

_server: Optional[QuestTCPServer] = None
_init_lock = threading.Lock()


def init() -> None:
    """Start the TCP listener. Idempotent."""
    global _server
    with _init_lock:
        if _server is not None:
            return
        port = int(os.environ.get("QUEST_XR_PORT", DEFAULT_PORT))
        host = os.environ.get("QUEST_XR_HOST", "0.0.0.0")
        srv = QuestTCPServer(host=host, port=port)
        srv.start()
        _server = srv


def _sample() -> Optional[dict]:
    return _server.get_latest() if _server is not None else None


# ---------------------------------------------------------------- body tracking
def is_body_data_available() -> bool:
    return _sample() is not None


def get_time_stamp_ns() -> int:
    s = _sample()
    return int(s["timeStampNs"]) if s is not None else time.time_ns()


def get_body_joints_pose():
    """Return a 24x7 SMPL pose list in Unity frame (scalar-last quaternion).

    Joints 0/12/22/23 are synthesized from the Quest head + two controllers; the
    remaining joints are zero-pos + identity quat (ignored downstream).
    """
    s = _sample()
    if s is None:
        # Match the shape downstream code expects even before the first frame.
        identity_row = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        return [list(identity_row) for _ in range(24)]
    pose = build_24x7_from_quest(s["head_pose"], s["left_pose"], s["right_pose"])
    return pose.tolist()


# ------------------------------------------------------------------- triggers
def get_left_trigger() -> float:
    s = _sample()
    return float(s["left_trigger"]) if s is not None else 0.0


def get_right_trigger() -> float:
    s = _sample()
    return float(s["right_trigger"]) if s is not None else 0.0


def get_left_grip() -> float:
    s = _sample()
    return float(s["left_grip"]) if s is not None else 0.0


def get_right_grip() -> float:
    s = _sample()
    return float(s["right_grip"]) if s is not None else 0.0


# -------------------------------------------------------------------- buttons
# A,B on the right controller (primary/secondary); X,Y on the left controller.
def get_A_button() -> bool:
    s = _sample()
    return bool(s["right_primary"]) if s is not None else False


def get_B_button() -> bool:
    s = _sample()
    return bool(s["right_secondary"]) if s is not None else False


def get_X_button() -> bool:
    s = _sample()
    return bool(s["left_primary"]) if s is not None else False


def get_Y_button() -> bool:
    s = _sample()
    return bool(s["left_secondary"]) if s is not None else False


def get_left_menu_button() -> bool:
    s = _sample()
    return bool(s["left_menu"]) if s is not None else False


def get_right_menu_button() -> bool:
    s = _sample()
    return bool(s["right_menu"]) if s is not None else False


# ----------------------------------------------------------------------- axes
def get_left_axis():
    s = _sample()
    if s is None:
        return (0.0, 0.0)
    return (float(s["left_axisX"]), float(s["left_axisY"]))


def get_right_axis():
    s = _sample()
    if s is None:
        return (0.0, 0.0)
    return (float(s["right_axisX"]), float(s["right_axisY"]))


def get_left_axis_click() -> bool:
    s = _sample()
    return bool(s["left_axis_click"]) if s is not None else False


def get_right_axis_click() -> bool:
    s = _sample()
    return bool(s["right_axis_click"]) if s is not None else False
