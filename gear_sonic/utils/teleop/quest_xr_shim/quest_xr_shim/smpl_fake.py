"""24x7 SMPL pose synthesis from 3 Quest tracking points.

The downstream consumer (``_process_3pt_pose`` in
``pico_manager_thread_server.py``) only reads four joints from the 24-row SMPL
array: ``[0=Pelvis, 12=Neck, 22=L-Wrist, 23=R-Wrist]``. We populate exactly
those and leave the rest as zeros + identity quaternion.

Pelvis is derived from the head: drop 0.7 m along Unity world Y (assumes the
operator is standing upright) and project the head orientation down to yaw-only
about Unity Y.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

PELVIS_DROP_Y_M = 0.7


def _identity_24x7() -> np.ndarray:
    out = np.zeros((24, 7), dtype=np.float32)
    out[:, 6] = 1.0  # qw = 1 for identity quaternion (scalar-last)
    return out


def _derive_pelvis(head_pose: list[float]) -> np.ndarray:
    pelvis = np.zeros(7, dtype=np.float32)
    pelvis[0] = head_pose[0]
    pelvis[1] = head_pose[1] - PELVIS_DROP_Y_M
    pelvis[2] = head_pose[2]

    quat_xyzw = np.asarray(head_pose[3:7], dtype=np.float64)
    if np.linalg.norm(quat_xyzw) > 1e-6:
        # Intrinsic yxz Euler decomposition: first angle is yaw about Unity Y.
        yaw, _, _ = R.from_quat(quat_xyzw).as_euler("yxz", degrees=False)
        yaw_quat = R.from_euler("yxz", [yaw, 0.0, 0.0], degrees=False).as_quat()
        pelvis[3:7] = yaw_quat.astype(np.float32)
    else:
        pelvis[6] = 1.0
    return pelvis


def build_24x7_from_quest(
    head_pose: list[float],
    left_pose: list[float],
    right_pose: list[float],
) -> np.ndarray:
    """Return a ``(24, 7)`` SMPL pose array in Unity frame (scalar-last quat).

    Each row is ``[x, y, z, qx, qy, qz, qw]``. Only joints 0/12/22/23 are
    meaningful; the others are filler.
    """
    out = _identity_24x7()
    out[12] = np.asarray(head_pose, dtype=np.float32)
    out[22] = np.asarray(left_pose, dtype=np.float32)
    out[23] = np.asarray(right_pose, dtype=np.float32)
    out[0] = _derive_pelvis(head_pose)
    return out
