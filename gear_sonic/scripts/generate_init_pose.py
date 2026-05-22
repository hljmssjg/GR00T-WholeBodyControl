"""Generate gear_sonic/data/autopilot_trajs/init_pose.json from the robot XML's
default qpos (the same state Backspace lands on in sim).

Usage:
    .venv_sim/bin/python gear_sonic/scripts/generate_init_pose.py

The script loads the robot's MuJoCo model, calls mj_resetData (Backspace
equivalent), reads the world-frame positions/orientations of pelvis +
left_wrist_yaw_link + right_wrist_yaw_link + torso_link, and writes them
relative to pelvis in the same format the PlannerStreamer override consumes:

  vr_3pt_position:    9 floats  [L-wrist xyz, R-wrist xyz, Neck xyz]
  vr_3pt_orientation: 12 floats [L-wrist wxyz, R-wrist wxyz, Neck wxyz]
  lh_joints:           7 floats (open hand defaults — all zeros)
  rh_joints:           7 floats (same)

The order matches gear_sonic/scripts/pico_manager_thread_server.py line ~224:
  Row 0: Left Wrist, Row 1: Right Wrist, Row 2: Neck.

Run once, commit the resulting init_pose.json. Re-run after model changes.
"""

from pathlib import Path
import json

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = Path(__file__).resolve().parent.parent.parent
XML = REPO / "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml"
OUT = REPO / "gear_sonic/data/autopilot_trajs/init_pose.json"


def main() -> int:
    if not XML.exists():
        print(f"ERROR: XML not found: {XML}")
        return 1
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)  # propagate qpos -> body world poses

    def body_pose(name: str):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise RuntimeError(f"body {name!r} not found")
        pos = np.array(data.xpos[bid], dtype=np.float64).copy()
        # xquat is mujoco's wxyz scalar-first quaternion
        quat_wxyz = np.array(data.xquat[bid], dtype=np.float64).copy()
        return pos, quat_wxyz

    pelvis_pos, pelvis_quat_wxyz = body_pose("pelvis")
    lw_pos, lw_quat_wxyz = body_pose("left_wrist_yaw_link")
    rw_pos, rw_quat_wxyz = body_pose("right_wrist_yaw_link")
    neck_pos, neck_quat_wxyz = body_pose("torso_link")

    # Convert pelvis_quat (wxyz) → scipy Rotation (xyzw) and invert.
    pelvis_q_xyzw = np.array(
        [pelvis_quat_wxyz[1], pelvis_quat_wxyz[2], pelvis_quat_wxyz[3], pelvis_quat_wxyz[0]]
    )
    pelvis_R = R.from_quat(pelvis_q_xyzw)
    pelvis_R_inv = pelvis_R.inv()

    def rel(pos, quat_wxyz):
        # Position: world-to-pelvis-frame translation.
        dp = pos - pelvis_pos
        rel_pos = pelvis_R_inv.apply(dp)
        # Orientation: pelvis^-1 * body, output as wxyz.
        body_R = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        rel_R = pelvis_R_inv * body_R
        rel_q_xyzw = rel_R.as_quat()
        rel_q_wxyz = np.array(
            [rel_q_xyzw[3], rel_q_xyzw[0], rel_q_xyzw[1], rel_q_xyzw[2]]
        )
        return rel_pos, rel_q_wxyz

    lw_rel_pos, lw_rel_q = rel(lw_pos, lw_quat_wxyz)
    rw_rel_pos, rw_rel_q = rel(rw_pos, rw_quat_wxyz)
    neck_rel_pos, neck_rel_q = rel(neck_pos, neck_quat_wxyz)

    # Order matches _process_3pt_pose's output rows: L-Wrist, R-Wrist, Neck.
    vr_3pt_position = (
        list(map(float, lw_rel_pos))
        + list(map(float, rw_rel_pos))
        + list(map(float, neck_rel_pos))
    )
    vr_3pt_orientation = (
        list(map(float, lw_rel_q))
        + list(map(float, rw_rel_q))
        + list(map(float, neck_rel_q))
    )

    # Hand joints — default "open" = zeros. compute_hand_joints_from_inputs
    # returns (1, 7) when no IK solver attached; flattened length 7.
    lh_joints = [0.0] * 7
    rh_joints = [0.0] * 7

    payload = {
        "vr_3pt_position": vr_3pt_position,
        "vr_3pt_orientation": vr_3pt_orientation,
        "lh_joints": lh_joints,
        "rh_joints": rh_joints,
        "_source": "generate_init_pose.py from g1_29dof_with_hand.xml mj_resetData",
    }

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
    print(f"Wrote {OUT}")
    print(f"  L-Wrist rel: pos={lw_rel_pos}, quat_wxyz={lw_rel_q}")
    print(f"  R-Wrist rel: pos={rw_rel_pos}, quat_wxyz={rw_rel_q}")
    print(f"  Neck    rel: pos={neck_rel_pos}, quat_wxyz={neck_rel_q}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
