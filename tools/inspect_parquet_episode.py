#!/usr/bin/env python3
"""Inspect one real-robot episode parquet file from a GR00T/SONIC run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


DEFAULT_COLUMNS = [
    "timestamp",
    "frame_index",
    "episode_index",
    "observation.state",
    "action.wbc",
    "observation.velocity",
    "observation.eef_state",
    "observation.motor_torque",
    "observation.motor_error",
    "diagnostic.motion_play",
    "diagnostic.state_age_ms",
    "diagnostic.camera_age_ms",
    "diagnostic.image_state_delta_ms",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("outputs/2026-06-12-15-58-12"),
        help="LeRobot dataset root.",
    )
    parser.add_argument("--episode", type=int, default=3, help="Episode index to inspect.")
    parser.add_argument("--frame", type=int, default=None, help="Frame index inside the episode.")
    parser.add_argument("--time", type=float, default=None, help="Nearest timestamp in seconds.")
    parser.add_argument("--columns", action="store_true", help="Print all parquet columns.")
    parser.add_argument("--schema", action="store_true", help="Print parquet dtypes.")
    parser.add_argument("--head", type=int, default=0, help="Print the first N scalar rows.")
    parser.add_argument(
        "--top-diff",
        type=int,
        default=12,
        help="Show the largest action.wbc - observation.state joint differences.",
    )
    parser.add_argument(
        "--all-joints",
        action="store_true",
        help="Print all 43 joints instead of only the largest differences.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def episode_path(dataset: Path, episode: int) -> Path:
    return dataset / "data" / "chunk-000" / f"episode_{episode:06d}.parquet"


def array_text(value: Any, precision: int = 4, max_items: int | None = None) -> str:
    array = np.asarray(value)
    if max_items is not None and array.size > max_items:
        flat = array.reshape(-1)
        shown = np.array2string(flat[:max_items], precision=precision, separator=", ")
        return f"{shown[:-1]}, ...] shape={array.shape}"
    return np.array2string(array, precision=precision, separator=", ")


def print_episode_summary(dataset: Path) -> None:
    episodes_path = dataset / "meta" / "episodes.jsonl"
    outcomes_path = dataset / "meta" / "episode_outcomes.jsonl"
    if not episodes_path.exists():
        return
    outcomes = {
        int(row["episode_index"]): row.get("outcome", "unknown")
        for row in load_jsonl(outcomes_path)
    } if outcomes_path.exists() else {}
    print("Episodes:")
    for row in load_jsonl(episodes_path):
        episode = int(row["episode_index"])
        length = int(row["length"])
        duration = length / 50.0
        task = row.get("tasks", [""])[0]
        print(
            f"  ep={episode} frames={length} duration={duration:.2f}s "
            f"outcome={outcomes.get(episode, 'unknown')} task={task}"
        )
    print()


def print_scalar_head(df: pd.DataFrame, rows: int) -> None:
    scalar_cols = [
        col
        for col in df.columns
        if not hasattr(df[col].iloc[0], "__len__") or isinstance(df[col].iloc[0], str)
    ]
    print(f"First {rows} scalar rows:")
    print(df[scalar_cols].head(rows).to_string(index=False))
    print()


def print_joint_table(
    names: list[str],
    actual: np.ndarray,
    target: np.ndarray,
    velocity: np.ndarray | None,
    top_diff: int,
    all_joints: bool,
) -> None:
    diff = target - actual
    if all_joints:
        indices = range(len(names))
        title = "All joints:"
    else:
        indices = np.argsort(np.abs(diff))[::-1][:top_diff]
        title = f"Top {top_diff} |action.wbc - observation.state| joints:"
    print(title)
    print(f"{'idx':>3}  {'joint':34s} {'actual':>10} {'wbc':>10} {'diff':>10} {'vel':>10}")
    for idx in indices:
        vel = velocity[idx] if velocity is not None and idx < len(velocity) else np.nan
        print(
            f"{idx:3d}  {names[idx]:34s} "
            f"{actual[idx]:10.4f} {target[idx]:10.4f} {diff[idx]:10.4f} {vel:10.4f}"
        )
    print()


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    info_path = dataset / "meta" / "info.json"
    if not info_path.exists():
        raise SystemExit(f"Missing dataset metadata: {info_path}")

    info = load_json(info_path)
    joint_names = info["features"]["observation.state"]["names"]
    parquet_path = episode_path(dataset, args.episode)
    if not parquet_path.exists():
        raise SystemExit(f"Missing parquet file: {parquet_path}")

    print(f"Dataset: {dataset}")
    print(f"Parquet: {parquet_path}")
    print(f"FPS: {info.get('fps')}  total_episodes: {info.get('total_episodes')}")
    print_episode_summary(dataset)

    df = pd.read_parquet(parquet_path)
    print(f"Loaded episode {args.episode}: rows={len(df)} columns={len(df.columns)}")
    print(f"Time range: {float(df['timestamp'].iloc[0]):.3f}s -> {float(df['timestamp'].iloc[-1]):.3f}s")
    print()

    if args.columns:
        print("Columns:")
        for col in df.columns:
            value = df[col].iloc[0]
            shape = getattr(value, "shape", None)
            length = len(value) if hasattr(value, "__len__") and not isinstance(value, str) else "-"
            print(f"  {col}: {type(value).__name__}, shape={shape}, len={length}")
        print()

    if args.schema:
        print("Dtypes:")
        print(df.dtypes.to_string())
        print()

    if args.head:
        print_scalar_head(df, args.head)

    if args.time is not None:
        row_index = int(np.argmin(np.abs(df["timestamp"].to_numpy(dtype=float) - args.time)))
    elif args.frame is not None:
        row_index = int(args.frame)
    else:
        row_index = 0
    if row_index < 0 or row_index >= len(df):
        raise SystemExit(f"Frame out of range: {row_index}, valid 0..{len(df)-1}")

    row = df.iloc[row_index]
    print(f"Selected row: local_frame={row_index}")
    print(f"  timestamp={float(row['timestamp']):.4f}s")
    print(f"  frame_index={int(row['frame_index'])} episode_index={int(row['episode_index'])}")
    print(f"  motion_play={row.get('diagnostic.motion_play', 'N/A')}")
    print(f"  state_age_ms={row.get('diagnostic.state_age_ms', np.nan):.3f}")
    print(f"  camera_age_ms={row.get('diagnostic.camera_age_ms', np.nan):.3f}")
    print(f"  image_state_delta_ms={row.get('diagnostic.image_state_delta_ms', np.nan):.3f}")
    print()

    actual = np.asarray(row["observation.state"], dtype=float)
    target = np.asarray(row["action.wbc"], dtype=float)
    velocity = np.asarray(row["observation.velocity"], dtype=float) if "observation.velocity" in df.columns else None
    print_joint_table(joint_names, actual, target, velocity, args.top_diff, args.all_joints)

    print("Other vectors:")
    for col in [
        "observation.eef_state",
        "observation.root_orientation",
        "observation.projected_gravity",
        "observation.base_angular_velocity",
        "observation.base_acceleration",
        "observation.motor_torque",
        "observation.motor_error",
        "action.motion_token",
    ]:
        if col in df.columns:
            print(f"  {col}: {array_text(row[col], max_items=16)}")


if __name__ == "__main__":
    main()
