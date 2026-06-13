#!/usr/bin/env python3
"""Render 43-DoF G1 + DEX3-1 kinematic poses from a prepared NPZ file."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess

import cv2
import mujoco
import numpy as np

CHINESE_FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--width", type=int, default=440)
    parser.add_argument("--height", type=int, default=400)
    return parser.parse_args()


def set_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: np.ndarray,
    joint_qpos_addresses: np.ndarray,
    root_quaternion: np.ndarray,
) -> None:
    mujoco.mj_resetData(model, data)
    # Keep the floating base at the nominal pelvis height from the MJCF.
    # The recorded log contains root orientation, but no world translation.
    data.qpos[:3] = model.qpos0[:3]
    quaternion = np.asarray(root_quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    data.qpos[3:7] = quaternion / norm if norm > 1e-8 else [1.0, 0.0, 0.0, 0.0]
    joint_values = np.asarray(joints, dtype=np.float64)
    if len(joint_values) != len(joint_qpos_addresses):
        raise ValueError(
            f"joint value/address count mismatch: {len(joint_values)} != "
            f"{len(joint_qpos_addresses)}"
        )
    data.qpos[joint_qpos_addresses] = joint_values
    # This is a kinematic replay. mj_forward updates transforms and contacts,
    # but does not integrate gravity, actuator forces, or contact dynamics.
    mujoco.mj_forward(model, data)


def tint_robot(model: mujoco.MjModel, original: np.ndarray, color: tuple[float, float, float]) -> None:
    model.geom_rgba[:] = original
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name == "floor":
            continue
        rgba = model.geom_rgba[geom_id]
        if rgba[3] <= 0:
            continue
        base = np.asarray(color, dtype=np.float32)
        luminance = float(np.mean(rgba[:3]))
        model.geom_rgba[geom_id, :3] = np.clip(base * (0.65 + 0.55 * luminance), 0, 1)


def add_panel_header(image: np.ndarray, timestamp: float) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (14, 14), (image.shape[1] - 14, 76), (12, 17, 24), -1)
    cv2.addWeighted(overlay, 0.78, image, 0.22, 0, image)
    cv2.putText(
        image,
        f"t = {timestamp:.2f} s",
        (28, 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (210, 218, 230),
        1,
        cv2.LINE_AA,
    )


def main() -> None:
    args = parse_args()
    payload = np.load(args.input)
    times = np.asarray(payload["times"], dtype=np.float64)
    actual = np.asarray(payload["actual"], dtype=np.float64)
    target = np.asarray(payload["target"], dtype=np.float64)
    root_quaternions = np.asarray(payload["root_quaternions"], dtype=np.float64)
    joint_names = [str(name) for name in payload["joint_names"]]

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joint_qpos_addresses = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"joint not found in MuJoCo model: {joint_name}")
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"expected hinge joint: {joint_name}")
        joint_qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
    if len(set(joint_qpos_addresses)) != len(joint_qpos_addresses):
        raise ValueError("duplicate MuJoCo qpos addresses in joint mapping")
    if len(joint_qpos_addresses) != model.nq - 7:
        raise ValueError(
            f"joint mapping covers {len(joint_qpos_addresses)} joints, "
            f"but model contains {model.nq - 7}"
        )
    joint_qpos_addresses = np.asarray(joint_qpos_addresses, dtype=np.int32)
    model.vis.global_.offwidth = args.width
    model.vis.global_.offheight = args.height
    model.vis.quality.offsamples = 1
    marker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "com_marker")
    if marker_id >= 0:
        model.site_rgba[marker_id, 3] = 0.0
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.78]
    camera.distance = 2.55
    camera.azimuth = 145.0
    camera.elevation = -10.0

    original_rgba = model.geom_rgba.copy()
    separator_width = 4
    output_width = args.width * 2 + separator_width
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temp_output = args.output.with_suffix(".encoding.mp4")
    if not CHINESE_FONT.exists():
        raise FileNotFoundError(f"Chinese font not found: {CHINESE_FONT}")
    target_title_x = args.width + separator_width + 28
    title_filter = (
        f"drawtext=fontfile='{CHINESE_FONT}':text='真机实测姿态':"
        "x=28:y=20:fontsize=23:fontcolor=0x96EEBC,"
        f"drawtext=fontfile='{CHINESE_FONT}':text='WBC 目标姿态':"
        f"x={target_title_x}:y=20:fontsize=23:fontcolor=0x76BEFF"
    )
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{output_width}x{args.height}",
            "-r",
            str(args.fps),
            "-i",
            "-",
            "-an",
            "-vf",
            title_filter,
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(temp_output),
        ],
        stdin=subprocess.PIPE,
    )

    try:
        for frame_index, timestamp in enumerate(times):
            set_pose(
                model,
                data,
                actual[frame_index],
                joint_qpos_addresses,
                root_quaternions[frame_index],
            )
            tint_robot(model, original_rgba, (0.66, 0.92, 0.78))
            renderer.update_scene(data, camera=camera)
            actual_image = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            add_panel_header(actual_image, timestamp)

            set_pose(
                model,
                data,
                target[frame_index],
                joint_qpos_addresses,
                root_quaternions[frame_index],
            )
            tint_robot(model, original_rgba, (0.52, 0.72, 1.0))
            renderer.update_scene(data, camera=camera)
            target_image = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            add_panel_header(target_image, timestamp)

            separator = np.full(
                (args.height, separator_width, 3), (35, 42, 54), dtype=np.uint8
            )
            frame = np.concatenate([actual_image, separator, target_image], axis=1)
            if frame.shape[1] != output_width:
                raise RuntimeError("unexpected rendered frame width")
            assert ffmpeg.stdin is not None
            ffmpeg.stdin.write(frame.tobytes())
    finally:
        model.geom_rgba[:] = original_rgba
        renderer.close()
        if ffmpeg.stdin is not None:
            ffmpeg.stdin.close()
        return_code = ffmpeg.wait()

    if return_code != 0:
        raise SystemExit(f"ffmpeg failed with exit code {return_code}")
    os.replace(temp_output, args.output)
    print(f"rendered {len(times)} frames to {args.output}")


if __name__ == "__main__":
    main()
