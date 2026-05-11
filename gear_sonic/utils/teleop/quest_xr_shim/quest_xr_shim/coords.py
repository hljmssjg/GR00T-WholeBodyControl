"""Pose-string parsing helpers for Quest Unity JSON frames.

Quest Unity APK serializes each tracked pose as a comma-separated string in the
form ``"x,y,z,qx,qy,qz,qw"`` (Unity Y-up left-handed frame, scalar-last
quaternion). An *untracked* device emits the sentinel ``"0,0,0,0,0,0,-1"``.

Downstream coordinate conversion (Unity -> robot) is handled by
``_compute_rel_transform`` in ``pico_manager_thread_server.py``; the shim only
needs to expose Unity-frame poses unchanged.
"""

from __future__ import annotations

UNTRACKED_QUAT = (0.0, 0.0, 0.0, -1.0)


def parse_pose_string(s: str) -> tuple[list[float], bool]:
    """Parse a Unity pose string. Returns ``(values, is_valid)``.

    ``values`` is always a 7-element ``[x, y, z, qx, qy, qz, qw]`` list,
    populated with zeros + identity quaternion on parse failure. ``is_valid`` is
    ``False`` for the untracked sentinel or any malformed input — callers should
    fall back to the previous valid frame in that case.
    """
    if not isinstance(s, str) or not s:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], False
    try:
        vals = [float(x) for x in s.split(",")]
    except ValueError:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], False
    if len(vals) != 7:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], False
    if tuple(vals[3:7]) == UNTRACKED_QUAT:
        return vals, False
    return vals, True
