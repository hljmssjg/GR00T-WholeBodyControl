#!/usr/bin/env python3
"""Standalone browser UI for annotating GR00T/SONIC real-robot episodes.

This tool treats the dataset as read-only. By default annotations are written
to ``annotations/<dataset-name>/`` under the repository root.

Usage:
    .venv_data_collection/bin/python tools/failure_annotation_app.py \
        --dataset outputs/2026-06-12-15-58-12

Then open http://127.0.0.1:8766/.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import re
import subprocess
import tempfile
import threading
import urllib.parse
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


APP_VERSION = 1
WRITE_LOCK = threading.Lock()
CACHE_LOCK = threading.Lock()
RENDER_LOCK = threading.Lock()
EPISODE_DETAIL_CACHE: dict[tuple[str, int, int], dict[str, Any]] = {}
REPO_ROOT = Path(__file__).resolve().parents[1]
G1_MODEL_DIR = REPO_ROOT / "gear_sonic" / "data" / "robot_model" / "model_data" / "g1"
G1_MJCF_PATH = G1_MODEL_DIR / "g1_29dof_with_hand.xml"
G1_SCENE_PATH = G1_MODEL_DIR / "scene_43dof.xml"
MUJOCO_RENDER_SCRIPT = REPO_ROOT / "tools" / "render_g1_episode_mujoco.py"
SIM_PYTHON = REPO_ROOT / ".venv_sim" / "bin" / "python"

PHASES = [
    "search",
    "approach",
    "reach",
    "grasp",
    "lift",
    "transport",
    "place",
    "verify",
    "recovery",
    "unknown",
]

ROOT_CAUSES = [
    "strategy.wrong_direction",
    "strategy.no_progress",
    "strategy.premature_transition",
    "perception.target_missing",
    "perception.state_misaligned",
    "latency.inference_slow",
    "latency.stale_input",
    "control.tracking_error",
    "control.unstable_action",
    "hardware.motor_fault",
    "hardware.collision_or_fall",
    "system.process_error",
    "system.communication_error",
    "operator.abort",
    "unknown",
]

PHASE_LABELS = {
    "search": "搜索目标",
    "approach": "接近目标",
    "reach": "伸手靠近",
    "grasp": "抓取",
    "lift": "抬起",
    "transport": "搬运",
    "place": "放置",
    "verify": "确认完成",
    "recovery": "恢复动作",
    "unknown": "未知阶段",
}

ROOT_CAUSE_LABELS = {
    "strategy.wrong_direction": "策略问题：运动方向错误",
    "strategy.no_progress": "策略问题：长时间没有进展",
    "strategy.premature_transition": "策略问题：过早切换任务阶段",
    "perception.target_missing": "感知问题：没有看见或丢失目标",
    "perception.state_misaligned": "感知问题：图像与机器人状态未对齐",
    "latency.inference_slow": "时延问题：模型推理过慢",
    "latency.stale_input": "时延问题：使用了陈旧输入",
    "control.tracking_error": "控制问题：实际关节没有跟上目标",
    "control.unstable_action": "控制问题：动作跳变或不稳定",
    "hardware.motor_fault": "硬件问题：电机故障或保护",
    "hardware.collision_or_fall": "硬件问题：碰撞、打滑或失稳",
    "system.process_error": "系统问题：进程报错或退出",
    "system.communication_error": "系统问题：通信中断或数据丢失",
    "operator.abort": "人工操作：操作者主动中止",
    "unknown": "未知：现有证据不足",
}

OUTCOME_LABELS = {
    "success": "成功",
    "failure": "失败",
    "discarded": "废弃",
    "unknown": "未知",
}

RECOVERABLE_LABELS = {
    "yes": "可以恢复",
    "no": "无法恢复",
    "unknown": "暂不确定",
}

SERIES_LABELS = {
    "diagnostic.state_age_ms": "机器人状态数据延迟（毫秒）",
    "diagnostic.camera_age_ms": "相机数据延迟（毫秒）",
    "diagnostic.image_state_delta_ms": "图像与机器人状态时间差（毫秒）",
    "diagnostic.sonic_age_ms": "SONIC 数据延迟（毫秒）",
    "diagnostic.cpp_state_index_delta": "C++ 状态序号跳变量",
    "tracking_rmse_rad": "关节目标跟踪误差（弧度均方根）",
    "max_abs_velocity": "最大关节速度绝对值",
    "max_abs_motor_torque": "最大电机估计力矩绝对值",
    "max_motor_temperature": "最高电机温度",
    "motor_error_count": "电机错误数量",
}

JOINT_TERM_LABELS = {
    "left": "左",
    "right": "右",
    "hip": "髋",
    "pitch": "俯仰",
    "roll": "横滚",
    "yaw": "偏航",
    "knee": "膝",
    "ankle": "踝",
    "waist": "腰",
    "shoulder": "肩",
    "elbow": "肘",
    "wrist": "腕",
    "hand": "手",
    "index": "食指",
    "middle": "中指",
    "thumb": "拇指",
}

SCALAR_SERIES = [
    "diagnostic.state_age_ms",
    "diagnostic.camera_age_ms",
    "diagnostic.image_state_delta_ms",
    "diagnostic.sonic_age_ms",
    "diagnostic.cpp_state_index_delta",
]

VECTOR_SERIES = {
    "tracking_rmse_rad": ("action.wbc", "observation.state", "rmse"),
    "max_abs_velocity": ("observation.velocity", None, "max_abs"),
    "max_abs_motor_torque": ("observation.motor_torque", None, "max_abs"),
    "max_motor_temperature": ("observation.motor_temperature", None, "max"),
    "motor_error_count": ("observation.motor_error", None, "nonzero_count"),
}

ARCHITECTURE_MODULES = [
    {
        "id": "camera",
        "label": "实时视觉输入",
        "subtitle": "ego RGB stream",
        "lane": "实时输入与高层决策",
        "kind": "input",
        "signals": ["diagnostic.camera_timestamp_s", "diagnostic.camera_age_ms"],
        "evidence": ["视频 mp4", "camera timestamp", "camera age"],
        "missing": ["VLA request 使用的 image_ref / image_hash"],
    },
    {
        "id": "robot_state",
        "label": "机器人本体状态",
        "subtitle": "q / dq / IMU / motor",
        "lane": "实时输入与高层决策",
        "kind": "input",
        "signals": [
            "diagnostic.cpp_state_index",
            "diagnostic.cpp_wall_timestamp_s",
            "diagnostic.state_age_ms",
            "max_abs_velocity",
            "max_abs_motor_torque",
            "max_motor_temperature",
            "motor_error_count",
        ],
        "evidence": ["observation.state", "observation.velocity", "IMU", "motor state"],
        "missing": ["C++ control tick 的结构化状态摘要"],
    },
    {
        "id": "prompt",
        "label": "任务语言",
        "subtitle": "prompt",
        "lane": "实时输入与高层决策",
        "kind": "input",
        "signals": ["task_prompt"],
        "evidence": ["meta/tasks.jsonl", "run_manifest prompt"],
        "missing": ["每次 VLA request 实际使用的 prompt 快照"],
    },
    {
        "id": "observation",
        "label": "Observation 构造",
        "subtitle": "时间对齐 + 状态分组",
        "lane": "实时输入与高层决策",
        "kind": "input",
        "signals": ["diagnostic.image_state_delta_ms", "diagnostic.exporter_timestamp_s"],
        "evidence": ["image/state delta", "projected_gravity", "prompt"],
        "missing": ["vla_request_id", "obs_ready_time", "输入图像帧引用"],
    },
    {
        "id": "vla",
        "label": "VLA 高层策略",
        "subtitle": "GR00T N1.7",
        "lane": "实时输入与高层决策",
        "kind": "policy",
        "signals": ["policy_total_ms", "inference_event_count"],
        "evidence": ["inference_trace/events.jsonl", "chunk npz"],
        "missing": ["完整 request start/end 与 raw policy action shape"],
    },
    {
        "id": "scheduler",
        "label": "Action Chunk 调度",
        "subtitle": "50 Hz step selection",
        "lane": "Token 到控制动作",
        "kind": "policy",
        "signals": [
            "publish_frame_index",
            "chunk_id",
            "chunk_index",
            "blend_alpha",
            "action.motion_token",
        ],
        "evidence": ["published_actions.jsonl", "selected_start_index", "ZMQ token_state"],
        "missing": ["C++ receive ack", "decode status", "receive timestamp"],
    },
    {
        "id": "sonic",
        "label": "SONIC 解码 / 控制策略",
        "subtitle": "token + 本体状态 -> q target",
        "lane": "Token 到控制动作",
        "kind": "control",
        "signals": ["action.motion_token", "action.wbc", "tracking_rmse_rad"],
        "evidence": ["motion token", "action.wbc", "hand target", "tracking RMSE"],
        "missing": ["decoder inference time", "raw/scaled last_action", "scale/default offset 分解日志"],
    },
    {
        "id": "low_control",
        "label": "低层控制与安全层",
        "subtitle": "PD / WBC / limit",
        "lane": "Token 到控制动作",
        "kind": "control",
        "signals": ["tracking_rmse_rad", "max_abs_motor_torque", "motor_error_count"],
        "evidence": ["target vs actual", "motor torque/error"],
        "missing": ["lowcmd", "safety flags", "limit/clamp 信息"],
    },
    {
        "id": "robot",
        "label": "G1 真机执行",
        "subtitle": "真实运动 / 接触 / 负载",
        "lane": "真机执行与反馈",
        "kind": "robot",
        "signals": ["tracking_rmse_rad", "max_abs_velocity", "max_motor_temperature"],
        "evidence": ["observation.state", "video", "motor diagnostics", "parquet/mp4"],
        "missing": ["接触/外力/跌倒事件结构化标记", "*-useful 因果链日志"],
    },
]

ARCHITECTURE_EDGES = [
    {
        "from": "camera",
        "to": "observation",
        "label": "camera -> obs",
        "metric": "diagnostic.camera_age_ms",
        "unit": "ms",
        "kind": "main",
    },
    {
        "from": "robot_state",
        "to": "observation",
        "label": "state -> obs",
        "metric": "diagnostic.state_age_ms",
        "unit": "ms",
        "kind": "main",
    },
    {
        "from": "prompt",
        "to": "observation",
        "label": "",
        "metric": "flow",
        "unit": "",
        "kind": "soft",
        "show_label": False,
    },
    {
        "from": "observation",
        "to": "vla",
        "label": "obs",
        "metric": "flow",
        "unit": "",
        "kind": "main",
    },
    {
        "from": "vla",
        "to": "scheduler",
        "label": "远程策略调用",
        "metric": "policy_total_ms",
        "unit": "ms",
        "kind": "main",
        "graph_label": "action chunk",
        "graph_metric": "chunk_id",
        "graph_unit": "",
    },
    {
        "from": "scheduler",
        "to": "sonic",
        "label": "ZMQ token_state[64]",
        "metric": "action.motion_token",
        "unit": "",
        "kind": "main",
    },
    {
        "from": "sonic",
        "to": "low_control",
        "label": "q target",
        "metric": "tracking_rmse_rad",
        "unit": "rad",
        "kind": "main",
    },
    {
        "from": "low_control",
        "to": "robot",
        "label": "电机错误",
        "metric": "motor_error_count",
        "unit": "",
        "kind": "main",
        "graph_label": "lowcmd",
        "graph_metric": "flow",
        "graph_unit": "",
    },
    {
        "from": "robot",
        "to": "robot_state",
        "label": "state feedback",
        "metric": "diagnostic.state_age_ms",
        "unit": "ms",
        "kind": "feedback",
    },
    {
        "from": "camera",
        "to": "robot_state",
        "label": "image/state 对齐",
        "metric": "diagnostic.image_state_delta_ms",
        "unit": "ms",
        "kind": "soft",
        "show_in_graph": False,
    },
]


def load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
    return rows


def atomic_write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def dataset_info(root: Path) -> dict[str, Any]:
    return load_json(root / "meta" / "info.json", {})


def camera_keys(info: dict[str, Any]) -> list[str]:
    return [
        key
        for key, value in info.get("features", {}).items()
        if isinstance(value, dict) and value.get("dtype") == "video"
    ]


def episode_paths(root: Path, info: dict[str, Any], episode_index: int) -> dict[str, Path]:
    chunk_size = int(info.get("chunks_size", 1000))
    chunk = episode_index // chunk_size
    result = {
        "parquet": root
        / "data"
        / f"chunk-{chunk:03d}"
        / f"episode_{episode_index:06d}.parquet"
    }
    for camera in camera_keys(info):
        result[camera] = (
            root
            / "videos"
            / f"chunk-{chunk:03d}"
            / camera
            / f"episode_{episode_index:06d}.mp4"
        )
    return result


def outcome_map(root: Path) -> dict[int, dict[str, Any]]:
    return {
        int(row["episode_index"]): row
        for row in load_jsonl(root / "meta" / "episode_outcomes.jsonl")
        if "episode_index" in row
    }


def annotation_map(path: Path) -> dict[int, dict[str, Any]]:
    return {
        int(row["episode_index"]): row
        for row in load_jsonl(path)
        if "episode_index" in row
    }


def process_log_root(dataset_root: Path) -> Path | None:
    candidate = dataset_root.with_name(dataset_root.name + "-process-logs")
    return candidate if candidate.exists() else None


def collect_episodes(
    root: Path, annotations_path: Path
) -> dict[str, Any]:
    info = dataset_info(root)
    episodes = load_jsonl(root / "meta" / "episodes.jsonl")
    outcomes = outcome_map(root)
    annotations = annotation_map(annotations_path)
    cameras = camera_keys(info)
    rows = []
    for episode in episodes:
        idx = int(episode["episode_index"])
        paths = episode_paths(root, info, idx)
        outcome = outcomes.get(idx, {})
        annotation = annotations.get(idx)
        rows.append(
            {
                "episode_index": idx,
                "length": episode.get("length"),
                "duration_s": (
                    float(episode.get("length", 0)) / float(info.get("fps", 1))
                    if info.get("fps")
                    else None
                ),
                "tasks": episode.get("tasks", []),
                "recorded_outcome": outcome.get("outcome", "unknown"),
                "available_cameras": [camera for camera in cameras if paths[camera].exists()],
                "annotated": annotation is not None,
                "annotation_outcome": (annotation or {}).get("outcome"),
                "failure_phase": (annotation or {}).get("failure_phase"),
                "root_causes": (annotation or {}).get("root_causes", []),
                "updated_at": (annotation or {}).get("updated_at"),
            }
        )
    return {
        "dataset": str(root),
        "annotations": str(annotations_path),
        "process_logs": str(process_log_root(root) or ""),
        "fps": info.get("fps"),
        "total_frames": info.get("total_frames"),
        "cameras": cameras,
        "phases": PHASES,
        "phase_labels": PHASE_LABELS,
        "root_causes": ROOT_CAUSES,
        "root_cause_labels": ROOT_CAUSE_LABELS,
        "outcome_labels": OUTCOME_LABELS,
        "recoverable_labels": RECOVERABLE_LABELS,
        "series_labels": SERIES_LABELS,
        "architecture_modules": ARCHITECTURE_MODULES,
        "architecture_edges": ARCHITECTURE_EDGES,
        "episodes": rows,
    }


def finite_number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def reduce_vector(value: Any, other: Any, operation: str) -> float | None:
    try:
        import numpy as np

        array = np.asarray(value, dtype=np.float64)
        if operation == "rmse":
            array = array - np.asarray(other, dtype=np.float64)
            return finite_number(np.sqrt(np.mean(np.square(array))))
        if operation == "max_abs":
            return finite_number(np.max(np.abs(array)))
        if operation == "max":
            return finite_number(np.max(array))
        if operation == "nonzero_count":
            valid = array[array >= 0]
            return float(np.count_nonzero(valid))
    except (TypeError, ValueError):
        return None
    return None


def downsample_indices(length: int, limit: int = 1200) -> list[int]:
    if length <= limit:
        return list(range(length))
    step = (length - 1) / (limit - 1)
    return sorted({round(i * step) for i in range(limit)})


def joint_label(name: str) -> str:
    parts = name.removesuffix("_joint").split("_")
    translated = [JOINT_TERM_LABELS.get(part, part) for part in parts]
    return "".join(translated)


def vector_rows(sampled: Any, column: str, width: int) -> list[list[float | None]]:
    if column not in sampled:
        return []
    rows = []
    for value in sampled[column]:
        try:
            values = list(value)
        except TypeError:
            values = []
        rows.append(
            [finite_number(values[index]) if index < len(values) else None for index in range(width)]
        )
    return rows


def load_episode_series(parquet_path: Path, info: dict[str, Any]) -> dict[str, Any]:
    try:
        import pandas as pd
    except ImportError as exc:
        raise RuntimeError("pandas/pyarrow are required to read episode parquet") from exc

    frame = pd.read_parquet(parquet_path)
    indices = downsample_indices(len(frame))
    sampled = frame.iloc[indices]
    if "timestamp" in sampled:
        times = [finite_number(value) or 0.0 for value in sampled["timestamp"]]
    else:
        times = [float(index) for index in indices]

    series: dict[str, list[float | None]] = {}
    for name in SCALAR_SERIES:
        if name in sampled:
            series[name] = [finite_number(value) for value in sampled[name]]

    for output_name, (left, right, operation) in VECTOR_SERIES.items():
        if left not in sampled or (right is not None and right not in sampled):
            continue
        values = []
        for row_idx in range(len(sampled)):
            other = sampled[right].iloc[row_idx] if right else None
            values.append(reduce_vector(sampled[left].iloc[row_idx], other, operation))
        series[output_name] = values

    features = info.get("features", {})
    joint_names = features.get("action.wbc", {}).get("names", [])
    if not isinstance(joint_names, list):
        joint_names = []
    joint_width = len(joint_names)
    token_shape = features.get("action.motion_token", {}).get("shape", [0])
    token_width = int(token_shape[0]) if token_shape else 0
    sample_columns: dict[str, list[Any]] = {}
    for name in (
        "frame_index",
        "index",
        "diagnostic.cpp_state_index",
        "diagnostic.cpp_wall_timestamp_s",
        "diagnostic.camera_timestamp_s",
        "diagnostic.exporter_timestamp_s",
    ):
        if name in sampled:
            values = []
            for value in sampled[name]:
                number = finite_number(value)
                values.append(number if number is not None else value)
            sample_columns[name] = values

    return {
        "frame_count": len(frame),
        "sample_count": len(indices),
        "source_indices": indices,
        "time_s": times,
        "series": series,
        "sample_columns": sample_columns,
        "actions": {
            "joint_names": joint_names,
            "joint_labels": [joint_label(name) for name in joint_names],
            "wbc_target": vector_rows(sampled, "action.wbc", joint_width),
            "joint_state": vector_rows(sampled, "observation.state", joint_width),
            "joint_velocity": vector_rows(sampled, "observation.velocity", joint_width),
            "motion_token": vector_rows(sampled, "action.motion_token", token_width),
            "motion_token_width": token_width,
        },
    }


def events_for_episode(root: Path, episode_index: int) -> list[dict[str, Any]]:
    events = load_jsonl(root / "meta" / "events.jsonl")
    return [row for row in events if int(row.get("episode_index", -1)) == episode_index]


def inference_for_episode(root: Path, episode_index: int) -> list[dict[str, Any]]:
    outcome = outcome_map(root).get(episode_index)
    logs = process_log_root(root)
    if not outcome or not logs:
        return []
    start = finite_number(outcome.get("started_at_unix_s"))
    end = finite_number(outcome.get("ended_at_unix_s"))
    if start is None or end is None:
        return []
    trace_path = logs / "inference_trace" / "events.jsonl"
    selected = []
    for row in load_jsonl(trace_path):
        timestamp = finite_number(row.get("time_unix_s"))
        if timestamp is None or timestamp < start or timestamp > end:
            continue
        event = row.get("event")
        if event in {
            "inference_chunk",
            "inference_error",
            "action_bound_violation",
            "worker_exception",
        }:
            item = dict(row)
            item["episode_time_s"] = timestamp - start
            selected.append(item)
    return selected


def published_actions_for_episode(root: Path, episode_index: int) -> list[dict[str, Any]]:
    outcome = outcome_map(root).get(episode_index)
    logs = process_log_root(root)
    if not outcome or not logs:
        return []
    start = finite_number(outcome.get("started_at_unix_s"))
    end = finite_number(outcome.get("ended_at_unix_s"))
    if start is None or end is None:
        return []
    trace_path = logs / "inference_trace" / "published_actions.jsonl"
    selected = []
    for row in load_jsonl(trace_path):
        timestamp = finite_number(row.get("time_unix_s"))
        if timestamp is None or timestamp < start or timestamp > end:
            continue
        selected.append(
            {
                "episode_time_s": timestamp - start,
                "time_unix_s": timestamp,
                "frame_index": row.get("frame_index"),
                "chunk_id": row.get("chunk_id"),
                "chunk_index": row.get("chunk_index"),
                "blend_alpha": row.get("blend_alpha"),
            }
        )
    return selected


def summarize_inference_events(events: list[dict[str, Any]]) -> dict[str, Any]:
    chunk_events = [row for row in events if row.get("event") == "inference_chunk"]
    inference_ms = [
        value
        for value in (finite_number(row.get("inference_ms")) for row in chunk_events)
        if value is not None
    ]
    return {
        "chunk_count": len(chunk_events),
        "error_count": sum(1 for row in events if row.get("event") != "inference_chunk"),
        "inference_ms_avg": (
            sum(inference_ms) / len(inference_ms) if inference_ms else None
        ),
        "inference_ms_max": max(inference_ms) if inference_ms else None,
    }


def episode_detail(root: Path, episode_index: int) -> dict[str, Any]:
    info = dataset_info(root)
    paths = episode_paths(root, info, episode_index)
    parquet_path = paths["parquet"]
    if not parquet_path.exists():
        raise KeyError(f"episode {episode_index} parquet not found")
    cache_key = (
        str(parquet_path),
        parquet_path.stat().st_mtime_ns,
        (root / "meta" / "info.json").stat().st_mtime_ns,
    )
    with CACHE_LOCK:
        cached_timeseries = EPISODE_DETAIL_CACHE.get(cache_key)
    if cached_timeseries is None:
        cached_timeseries = load_episode_series(parquet_path, info)
        with CACHE_LOCK:
            EPISODE_DETAIL_CACHE.clear()
            EPISODE_DETAIL_CACHE[cache_key] = cached_timeseries
    outcomes = outcome_map(root)
    inference_events = inference_for_episode(root, episode_index)
    return {
        "episode_index": episode_index,
        "recorded_outcome": outcomes.get(episode_index, {}),
        "events": events_for_episode(root, episode_index),
        "inference_events": inference_events,
        "inference_summary": summarize_inference_events(inference_events),
        "published_actions": published_actions_for_episode(root, episode_index),
        "timeseries": cached_timeseries,
    }


def empty_annotation(
    root: Path, episode_index: int, duration_s: float | None
) -> dict[str, Any]:
    recorded = outcome_map(root).get(episode_index, {})
    return {
        "schema_version": APP_VERSION,
        "dataset": root.name,
        "episode_index": episode_index,
        "recorded_outcome": recorded.get("outcome", "unknown"),
        "outcome": recorded.get("outcome", "unknown"),
        "failure_phase": "unknown",
        "anomaly_time_s": None,
        "anomaly_end_time_s": None,
        "root_causes": [],
        "root_cause_confidence": None,
        "visible_symptom": "",
        "root_cause_notes": "",
        "suggested_correction": "",
        "recoverable": "unknown",
        "phases": [],
        "tags": [],
        "annotator": "",
        "notes": "",
        "episode_duration_s": duration_s,
    }


def validate_annotation(
    payload: dict[str, Any], episode_index: int, duration_s: float | None
) -> dict[str, Any]:
    result = dict(payload)
    result["schema_version"] = APP_VERSION
    result["episode_index"] = episode_index

    if result.get("outcome") not in {"success", "failure", "discarded", "unknown"}:
        raise ValueError("outcome must be success, failure, discarded, or unknown")
    if result.get("failure_phase", "unknown") not in PHASES:
        raise ValueError("invalid failure_phase")
    if result.get("recoverable", "unknown") not in {"yes", "no", "unknown"}:
        raise ValueError("recoverable must be yes, no, or unknown")

    root_causes = result.get("root_causes") or []
    if not isinstance(root_causes, list) or any(cause not in ROOT_CAUSES for cause in root_causes):
        raise ValueError("invalid root_causes")
    result["root_causes"] = sorted(set(root_causes))

    for key in ("anomaly_time_s", "anomaly_end_time_s", "root_cause_confidence"):
        value = result.get(key)
        if value in ("", None):
            result[key] = None
            continue
        number = finite_number(value)
        if number is None:
            raise ValueError(f"{key} must be a finite number")
        result[key] = number

    if result["root_cause_confidence"] is not None and not (
        0 <= result["root_cause_confidence"] <= 1
    ):
        raise ValueError("root_cause_confidence must be between 0 and 1")

    max_time = duration_s if duration_s is not None else float("inf")
    for key in ("anomaly_time_s", "anomaly_end_time_s"):
        value = result[key]
        if value is not None and not (0 <= value <= max_time + 0.1):
            raise ValueError(f"{key} must be within the episode duration")
    if (
        result["anomaly_time_s"] is not None
        and result["anomaly_end_time_s"] is not None
        and result["anomaly_end_time_s"] < result["anomaly_time_s"]
    ):
        raise ValueError("anomaly_end_time_s must not precede anomaly_time_s")

    phases = result.get("phases") or []
    if not isinstance(phases, list):
        raise ValueError("phases must be a list")
    clean_phases = []
    for index, phase in enumerate(phases):
        if not isinstance(phase, dict) or phase.get("phase") not in PHASES:
            raise ValueError(f"invalid phase row {index}")
        start = finite_number(phase.get("start_s"))
        end = finite_number(phase.get("end_s"))
        if start is None or end is None or start < 0 or end < start or end > max_time + 0.1:
            raise ValueError(f"invalid time range in phase row {index}")
        clean_phases.append(
            {
                "phase": phase["phase"],
                "start_s": start,
                "end_s": end,
                "note": str(phase.get("note", "")).strip(),
            }
        )
    result["phases"] = sorted(clean_phases, key=lambda row: row["start_s"])

    tags = result.get("tags") or []
    if isinstance(tags, str):
        tags = [item.strip() for item in tags.split(",") if item.strip()]
    if not isinstance(tags, list):
        raise ValueError("tags must be a list")
    result["tags"] = sorted({str(item).strip() for item in tags if str(item).strip()})

    for key in (
        "visible_symptom",
        "root_cause_notes",
        "suggested_correction",
        "annotator",
        "notes",
    ):
        result[key] = str(result.get(key, "")).strip()
    result["episode_duration_s"] = duration_s
    return result


def save_annotation(
    root: Path,
    annotations_path: Path,
    history_path: Path,
    episode_index: int,
    payload: dict[str, Any],
) -> dict[str, Any]:
    info = dataset_info(root)
    episodes = load_jsonl(root / "meta" / "episodes.jsonl")
    episode = next(
        (row for row in episodes if int(row.get("episode_index", -1)) == episode_index),
        None,
    )
    if episode is None:
        raise KeyError(f"episode {episode_index} not found")
    fps = finite_number(info.get("fps"))
    duration_s = float(episode.get("length", 0)) / fps if fps else None
    clean = validate_annotation(payload, episode_index, duration_s)
    clean["dataset"] = root.name
    clean["recorded_outcome"] = outcome_map(root).get(episode_index, {}).get(
        "outcome", "unknown"
    )
    clean["updated_at"] = datetime.now(timezone.utc).isoformat()

    with WRITE_LOCK:
        annotations = annotation_map(annotations_path)
        previous = annotations.get(episode_index)
        annotations[episode_index] = clean
        atomic_write_jsonl(
            annotations_path,
            [annotations[index] for index in sorted(annotations)],
        )
        append_jsonl(
            history_path,
            {
                "saved_at": clean["updated_at"],
                "episode_index": episode_index,
                "previous": previous,
                "annotation": clean,
            },
        )
    return clean


def rendered_video_path(render_dir: Path, episode_index: int) -> Path:
    return render_dir / f"episode_{episode_index:06d}.mp4"


def render_is_current(
    video_path: Path,
    parquet_path: Path,
) -> bool:
    if not video_path.exists() or video_path.stat().st_size == 0:
        return False
    dependencies = [parquet_path, G1_MJCF_PATH, G1_SCENE_PATH, MUJOCO_RENDER_SCRIPT]
    output_mtime = video_path.stat().st_mtime_ns
    return all(path.exists() and path.stat().st_mtime_ns <= output_mtime for path in dependencies)


def prepare_mujoco_render_input(
    parquet_path: Path,
    destination: Path,
    output_fps: float,
    joint_names: list[str],
) -> int:
    import numpy as np
    import pandas as pd

    frame = pd.read_parquet(
        parquet_path,
        columns=[
            "timestamp",
            "observation.state",
            "action.wbc",
            "observation.root_orientation",
        ],
    )
    source_times = np.asarray(frame["timestamp"], dtype=np.float64)
    if not len(source_times):
        raise ValueError("episode has no frames")
    duration = float(source_times[-1])
    render_times = np.arange(0.0, duration + 1e-9, 1.0 / output_fps)
    indices = np.searchsorted(source_times, render_times, side="left")
    indices = np.clip(indices, 0, len(source_times) - 1)
    actual = np.stack(frame["observation.state"].iloc[indices].to_numpy())
    target = np.stack(frame["action.wbc"].iloc[indices].to_numpy())
    if actual.shape[1] != 43 or target.shape[1] != 43:
        raise ValueError(
            f"expected 43-DoF G1 + DEX3-1 data, got {actual.shape[1]} and {target.shape[1]}"
        )
    if len(joint_names) != actual.shape[1] or len(set(joint_names)) != len(joint_names):
        raise ValueError("dataset joint names are missing, duplicated, or dimensionally invalid")
    root_quaternions = np.stack(
        frame["observation.root_orientation"].iloc[indices].to_numpy()
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        destination,
        times=render_times,
        actual=actual,
        target=target,
        root_quaternions=root_quaternions,
        joint_names=np.asarray(joint_names),
    )
    return len(render_times)


def generate_mujoco_render(
    root: Path,
    render_dir: Path,
    episode_index: int,
) -> dict[str, Any]:
    info = dataset_info(root)
    state_feature = info.get("features", {}).get("observation.state", {})
    action_feature = info.get("features", {}).get("action.wbc", {})
    joint_names = state_feature.get("names")
    if not isinstance(joint_names, list) or action_feature.get("names") != joint_names:
        raise ValueError("observation.state and action.wbc must share named joints")
    parquet_path = episode_paths(root, info, episode_index)["parquet"]
    if not parquet_path.exists():
        raise KeyError(f"episode {episode_index} parquet not found")
    video_path = rendered_video_path(render_dir, episode_index)
    if render_is_current(video_path, parquet_path):
        return {"ready": True, "cached": True, "video": str(video_path)}
    if not SIM_PYTHON.exists():
        raise RuntimeError(f"MuJoCo Python environment not found: {SIM_PYTHON}")

    render_dir.mkdir(parents=True, exist_ok=True)
    input_path = render_dir / f".episode_{episode_index:06d}.render_input.npz"
    output_fps = 12.0
    with RENDER_LOCK:
        if render_is_current(video_path, parquet_path):
            return {"ready": True, "cached": True, "video": str(video_path)}
        frame_count = prepare_mujoco_render_input(
            parquet_path, input_path, output_fps, joint_names
        )
        environment = os.environ.copy()
        environment["MUJOCO_GL"] = "egl"
        environment["XDG_CACHE_HOME"] = str(render_dir / ".cache")
        try:
            result = subprocess.run(
                [
                    str(SIM_PYTHON),
                    str(MUJOCO_RENDER_SCRIPT),
                    "--input",
                    str(input_path),
                    "--model",
                    str(G1_SCENE_PATH),
                    "--output",
                    str(video_path),
                    "--fps",
                    str(output_fps),
                ],
                cwd=REPO_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=900,
                check=False,
            )
        finally:
            input_path.unlink(missing_ok=True)
        if result.returncode != 0:
            message = (result.stderr or result.stdout or "unknown render error").strip()
            raise RuntimeError(message[-2000:])
    return {
        "ready": True,
        "cached": False,
        "video": str(video_path),
        "frame_count": frame_count,
    }


class AnnotationHandler(BaseHTTPRequestHandler):
    dataset_root = Path()
    annotations_path = Path()
    history_path = Path()
    render_dir = Path()

    def log_message(self, fmt: str, *args: Any) -> None:
        message = fmt % args
        if "/video/" in message and " 206 " in message:
            return
        super().log_message(fmt, *args)

    def send_json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, status: int, message: str) -> None:
        self.send_json({"error": message}, status)

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self) -> None:  # noqa: N802
        path = urllib.parse.urlparse(self.path).path
        if path in {"/", "/index.html"}:
            body = INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/episodes":
            try:
                self.send_json(collect_episodes(self.dataset_root, self.annotations_path))
            except Exception as exc:  # noqa: BLE001
                self.send_error_json(500, str(exc))
            return
        match = re.match(r"^/api/episodes/(\d+)$", path)
        if match:
            try:
                self.send_json(episode_detail(self.dataset_root, int(match.group(1))))
            except KeyError as exc:
                self.send_error_json(404, str(exc))
            except Exception as exc:  # noqa: BLE001
                self.send_error_json(500, str(exc))
            return
        match = re.match(r"^/api/annotations/(\d+)$", path)
        if match:
            episode_index = int(match.group(1))
            annotations = annotation_map(self.annotations_path)
            annotation = annotations.get(episode_index)
            if annotation is None:
                episodes = collect_episodes(self.dataset_root, self.annotations_path)["episodes"]
                episode = next(
                    (row for row in episodes if row["episode_index"] == episode_index),
                    None,
                )
                if episode is None:
                    self.send_error_json(404, f"episode {episode_index} not found")
                    return
                annotation = empty_annotation(
                    self.dataset_root, episode_index, episode.get("duration_s")
                )
            self.send_json(annotation)
            return
        match = re.match(r"^/api/render/(\d+)$", path)
        if match:
            episode_index = int(match.group(1))
            info = dataset_info(self.dataset_root)
            parquet_path = episode_paths(self.dataset_root, info, episode_index)["parquet"]
            video_path = rendered_video_path(self.render_dir, episode_index)
            self.send_json(
                {
                    "ready": render_is_current(video_path, parquet_path),
                    "video_url": f"/render/g1/{episode_index}.mp4",
                }
            )
            return
        match = re.match(r"^/render/g1/(\d+)\.mp4$", path)
        if match:
            video_path = rendered_video_path(self.render_dir, int(match.group(1)))
            if not video_path.exists():
                self.send_error_json(404, "MuJoCo render not found")
                return
            self.stream_file(video_path)
            return
        match = re.match(r"^/video/([^/]+)/(\d+)\.mp4$", path)
        if match:
            self.serve_video(urllib.parse.unquote(match.group(1)), int(match.group(2)))
            return
        self.send_error_json(404, f"not found: {path}")

    def do_POST(self) -> None:  # noqa: N802
        path = urllib.parse.urlparse(self.path).path
        render_match = re.match(r"^/api/render/(\d+)$", path)
        if render_match:
            try:
                result = generate_mujoco_render(
                    self.dataset_root,
                    self.render_dir,
                    int(render_match.group(1)),
                )
                result["video_url"] = f"/render/g1/{int(render_match.group(1))}.mp4"
                self.send_json(result)
            except KeyError as exc:
                self.send_error_json(404, str(exc))
            except Exception as exc:  # noqa: BLE001
                self.send_error_json(500, str(exc))
            return
        match = re.match(r"^/api/annotations/(\d+)$", path)
        if not match:
            self.send_error_json(404, f"not found: {path}")
            return
        try:
            saved = save_annotation(
                self.dataset_root,
                self.annotations_path,
                self.history_path,
                int(match.group(1)),
                self.read_json_body(),
            )
            self.send_json(saved)
        except KeyError as exc:
            self.send_error_json(404, str(exc))
        except (ValueError, json.JSONDecodeError) as exc:
            self.send_error_json(400, str(exc))
        except Exception as exc:  # noqa: BLE001
            self.send_error_json(500, str(exc))

    def serve_video(self, camera: str, episode_index: int) -> None:
        info = dataset_info(self.dataset_root)
        if camera not in camera_keys(info):
            self.send_error_json(404, f"unknown camera: {camera}")
            return
        video_path = episode_paths(self.dataset_root, info, episode_index).get(camera)
        if video_path is None or not video_path.exists():
            self.send_error_json(404, "video not found")
            return
        self.stream_file(video_path)

    def stream_file(self, path: Path) -> None:
        file_size = path.stat().st_size
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        range_header = self.headers.get("Range")
        if not range_header:
            start, end, status = 0, file_size - 1, HTTPStatus.OK
        else:
            match = re.match(r"bytes=(\d*)-(\d*)", range_header)
            if not match:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.end_headers()
                return
            start_text, end_text = match.groups()
            if not start_text:
                suffix = int(end_text)
                start, end = max(0, file_size - suffix), file_size - 1
            else:
                start = int(start_text)
                end = int(end_text) if end_text else file_size - 1
            status = HTTPStatus.PARTIAL_CONTENT
        if start < 0 or start >= file_size or end < start or end >= file_size:
            self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
            self.send_header("Content-Range", f"bytes */{file_size}")
            self.end_headers()
            return
        length = end - start + 1
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(length))
        if status == HTTPStatus.PARTIAL_CONTENT:
            self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
        self.end_headers()
        with path.open("rb") as handle:
            handle.seek(start)
            remaining = length
            while remaining:
                data = handle.read(min(64 * 1024, remaining))
                if not data:
                    break
                self.wfile.write(data)
                remaining -= len(data)


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>真机失败标注台</title>
<style>
:root{--bg:#0d1016;--panel:#161b24;--panel2:#1d2430;--border:#303a49;--text:#e8edf5;
--muted:#97a3b6;--accent:#57a0ff;--ok:#43cb91;--danger:#ff6675;--warn:#f3b84b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);
font:14px system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
button,input,select,textarea{font:inherit}button{cursor:pointer}.top{position:sticky;top:0;z-index:5;
background:#0d1016ed;border-bottom:1px solid var(--border);padding:12px 18px}
.top h1{font-size:18px;margin:0}.meta{color:var(--muted);font-size:12px;margin-top:5px}
.layout{display:grid;grid-template-columns:260px minmax(0,1fr);height:calc(100vh - 67px)}
.sidebar{border-right:1px solid var(--border);overflow:auto;padding:12px}.filters{display:flex;gap:7px;margin-bottom:10px}
.filters select,.filters input{min-width:0;background:var(--panel2);color:var(--text);border:1px solid var(--border);
border-radius:6px;padding:7px}.filters input{width:100%}.episode{background:var(--panel);border:1px solid var(--border);
border-radius:8px;padding:10px;margin-bottom:8px;cursor:pointer}.episode:hover,.episode.active{border-color:var(--accent)}
.episode-title{display:flex;justify-content:space-between;font-weight:650}.badges{display:flex;gap:5px;flex-wrap:wrap;margin-top:7px}
.badge{padding:2px 6px;border-radius:9px;font-size:11px;background:var(--panel2);color:var(--muted)}
.badge.success{color:var(--ok)}.badge.failure{color:var(--danger)}.badge.annotated{color:var(--accent)}
.main{overflow:auto;padding:16px}.empty{height:100%;display:grid;place-items:center;color:var(--muted)}
.workspace{display:none}.workspace.open{display:block}.headrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.headrow h2{margin:0}.camera-tabs button,.smallbtn{background:var(--panel2);color:var(--text);border:1px solid var(--border);
border-radius:6px;padding:6px 9px}.camera-tabs button.active{background:var(--accent);border-color:var(--accent)}
.architecture-panel{padding:0;overflow:hidden}.arch-head{display:flex;justify-content:space-between;gap:12px;align-items:center;
padding:12px;border-bottom:1px solid var(--border)}.arch-head h3{margin:0}.arch-note{font-size:12px;color:var(--muted)}
.latency-strip{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:8px;padding:10px 12px;
border-bottom:1px solid var(--border);background:#10141b}.latency-chip{border:1px solid var(--border);border-radius:7px;
padding:7px;background:var(--panel2)}.latency-chip strong{display:block;font-size:12px}.latency-chip span{font-size:15px;
font-variant-numeric:tabular-nums}.latency-chip.ok span{color:var(--ok)}.latency-chip.warn span{color:var(--warn)}
.latency-chip.bad span{color:var(--danger)}.latency-chip.missing span{color:var(--muted)}
.arch-map{overflow:auto;background:#0f131a}.arch-stage{position:relative;width:1180px;height:430px;margin:0 auto}
.arch-svg{position:absolute;inset:0;width:1180px;height:430px;pointer-events:none;z-index:1}.arch-lane-band{position:absolute;left:12px;right:12px;
border:1px solid #252f3d;border-radius:14px;background:#121821;z-index:0}.arch-lane-label{position:absolute;left:24px;color:var(--muted);
font-size:12px;font-weight:700;z-index:4;background:#121821;padding:0 8px;border-radius:6px}.arch-node{position:absolute;text-align:left;background:var(--panel2);color:var(--text);
border:1px solid var(--border);border-radius:8px;padding:9px;min-height:68px;z-index:2}
.arch-node:hover,.arch-node.active{border-color:var(--accent);z-index:2}.arch-node.input{box-shadow:inset 3px 0 #5b9bd5}
.arch-node.policy{box-shadow:inset 3px 0 #e0a33f}.arch-node.control{box-shadow:inset 3px 0 #48a868}
.arch-node.robot{box-shadow:inset 3px 0 #b98cff}.arch-node.log{box-shadow:inset 3px 0 #9da9bc}
.arch-title{font-weight:700;font-size:13px}.arch-sub{color:var(--muted);font-size:12px;margin-top:2px}
.edge-path{fill:none;stroke:#6f8195;stroke-width:2.2}.edge-path.main{stroke:#aeb9c8}.edge-path.soft{stroke:#768696;stroke-width:1.8}
.edge-path.feedback{stroke:#b98cff;stroke-width:2.1;stroke-dasharray:7 5}.edge-path.log{stroke:#7d8997;stroke-width:1.7;stroke-dasharray:5 5}
.edge-label rect{fill:#151b24;stroke:#303a49;rx:6;ry:6}.edge-label text{fill:#c7d0dd;font-size:11px;
font-family:ui-monospace,SFMono-Regular,Consolas,monospace}.edge-label.ok text{fill:var(--ok)}
.edge-label.warn text{fill:var(--warn)}.edge-label.bad text{fill:var(--danger)}.edge-label.missing text{fill:var(--muted)}
.arch-legend{position:absolute;left:24px;bottom:12px;color:var(--muted);font-size:11px;display:flex;gap:16px;align-items:center}
.legend-line{display:inline-block;width:34px;height:0;border-top:2px solid #aeb9c8;vertical-align:middle;margin-right:5px}
.legend-line.feedback{border-top-color:#b98cff;border-top-style:dashed}
.cursor-context{display:flex;gap:12px;flex-wrap:wrap;padding:8px 12px;border-top:1px solid var(--border);
background:#10141b;color:var(--muted);font-size:12px}.cursor-context strong{color:var(--text);font-weight:600}
.module-details{display:grid;grid-template-columns:1.1fr .9fr;gap:10px;padding:0 12px 12px}.module-card{background:#10141b;
border:1px solid var(--border);border-radius:7px;padding:10px}.module-card h4{margin:0 0 8px;font-size:13px}
.kv{display:grid;grid-template-columns:minmax(130px,.7fr) minmax(0,1fr);gap:5px 8px;font-size:12px}.kv div:nth-child(odd){color:var(--muted)}
.tagrow{display:flex;gap:6px;flex-wrap:wrap}.tag{font-size:11px;color:var(--muted);background:var(--panel2);border:1px solid var(--border);
border-radius:999px;padding:2px 7px}.tag.missing{color:var(--warn);border-color:#6f5630}
.pipeline-panels{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;padding:0 12px 12px}
.pipeline-card{background:#10141b;border:1px solid var(--border);border-radius:7px;padding:10px;min-width:0}
.pipeline-card h4{margin:0 0 7px;font-size:13px}.pipeline-card .hint{color:var(--muted);font-size:11px;margin-top:5px}
.pipeline-canvas{display:block;width:100%;height:300px;background:#0a0d13;border:1px solid #253040;border-radius:6px}
.pipeline-canvas.chunk-canvas{height:390px}
.chunk-scroll{height:390px;overflow:auto;border:1px solid #253040;border-radius:6px;background:#0a0d13}
.chunk-scroll .pipeline-canvas{border:0;border-radius:0}
.camera-video{display:block;width:100%;height:420px;max-height:none;object-fit:contain;background:#000;border:1px solid #253040;border-radius:6px}
.pose-crop{position:relative;width:min(100%,560px);aspect-ratio:1.1/1;margin:0 auto;overflow:hidden;background:#080a0e;border:1px solid #253040;border-radius:6px}
.pose-crop video{position:absolute;top:0;height:100%;width:200%;max-width:none;object-fit:fill;pointer-events:none}
.pose-crop.actual video{left:0}.pose-crop.target video{left:-100%}.pose-empty{display:grid;place-items:center;height:100%;color:var(--muted);font-size:12px}
.columns{display:grid;grid-template-columns:minmax(420px,1.25fr) minmax(360px,.75fr);gap:14px;margin-top:12px}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:9px;padding:12px;margin-bottom:12px}
.panel h3{font-size:14px;margin:0 0 10px}.video{width:100%;max-height:58vh;background:#000;border-radius:6px}
.timebar{display:flex;gap:8px;align-items:center;margin-top:8px}.timebar input{flex:1}.time{font-variant-numeric:tabular-nums;color:var(--muted)}
.quick{display:flex;gap:7px;flex-wrap:wrap;margin-top:8px}.quick button{background:var(--panel2);color:var(--text);
border:1px solid var(--border);border-radius:6px;padding:6px 9px}
.action-controls{display:flex;gap:9px;flex-wrap:wrap;margin-bottom:9px}.action-controls label{display:flex;
align-items:center;gap:6px;color:var(--muted);font-size:12px}.action-controls select{background:var(--panel2);
color:var(--text);border:1px solid var(--border);border-radius:6px;padding:6px;max-width:260px}
.action-chart{display:block;width:100%;height:180px}.legend{display:flex;gap:12px;flex-wrap:wrap;
font-size:11px;color:var(--muted);margin:5px 0}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px}
.sim-video{display:block;width:100%;max-height:520px;background:#080a0e;border:1px solid var(--border);
border-radius:7px}.sim-toolbar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
.sim-toolbar button{background:var(--accent);color:white;border:0;border-radius:6px;padding:7px 12px}
.sim-toolbar button:disabled{opacity:.5}.sim-hint{font-size:11px;color:var(--muted);margin-top:6px}
.charts{display:grid;gap:8px}.chartbox{background:#10141b;border:1px solid var(--border);border-radius:6px;padding:6px}
.chart-title{font-size:11px;color:var(--muted);margin-bottom:3px}.chartbox canvas{display:block;width:100%;height:90px}
.event-list{max-height:220px;overflow:auto;font:12px ui-monospace,monospace}.event{padding:5px;border-bottom:1px solid #252d39}
.event button{color:var(--accent);background:none;border:0;padding:0;margin-right:7px}.formgrid{display:grid;
grid-template-columns:1fr 1fr;gap:9px}.field{display:flex;flex-direction:column;gap:4px}.field.full{grid-column:1/-1}
.field label{font-size:12px;color:var(--muted)}.field input,.field select,.field textarea{background:var(--panel2);
color:var(--text);border:1px solid var(--border);border-radius:6px;padding:7px}.field textarea{min-height:62px;resize:vertical}
.causes{display:grid;grid-template-columns:1fr 1fr;gap:5px}.cause{font-size:12px;color:var(--muted)}
.phase-row{display:grid;grid-template-columns:1.1fr .7fr .7fr 1.4fr auto;gap:6px;margin-bottom:6px}
.phase-row input,.phase-row select{min-width:0;background:var(--panel2);color:var(--text);border:1px solid var(--border);
border-radius:5px;padding:6px}.remove{background:none;color:var(--danger);border:1px solid #6b3340;border-radius:5px}
.save{width:100%;padding:9px;background:var(--accent);color:white;border:0;border-radius:7px;font-weight:650}
.save:disabled{opacity:.5}.status{font-size:12px;color:var(--muted);text-align:center;margin-top:7px}
@media(max-width:980px){.layout{grid-template-columns:1fr;height:auto}.sidebar{border-right:0;border-bottom:1px solid var(--border);
max-height:300px}.columns,.module-details,.pipeline-panels{grid-template-columns:1fr}.arch-stage{margin:0}}
</style>
</head>
<body>
<div class="top"><h1>真机失败标注台</h1><div id="datasetMeta" class="meta">加载中...</div></div>
<div class="layout">
  <aside class="sidebar">
    <div class="filters"><input id="search" placeholder="搜索回合编号或任务">
      <select id="outcomeFilter"><option value="">全部结果</option>
      <option value="success">成功</option><option value="failure">失败</option>
      <option value="discarded">废弃</option><option value="unknown">未知</option></select></div>
    <div id="episodeList"></div>
  </aside>
  <main class="main">
    <div id="empty" class="empty">选择一个回合开始标注</div>
    <div id="workspace" class="workspace">
      <div class="headrow"><h2 id="episodeTitle"></h2><span id="recordedBadge" class="badge"></span>
        <div id="cameraTabs" class="camera-tabs"></div></div>
      <div class="panel architecture-panel">
        <div class="arch-head">
          <h3>Eval Pipeline 模块图</h3>
          <div class="arch-note">点击模块查看当前视频时间附近的数据；边上的延迟来自已有 parquet / process logs</div>
        </div>
        <div id="latencyStrip" class="latency-strip"></div>
        <div id="archMap" class="arch-map"></div>
        <div id="cursorContext" class="cursor-context"></div>
        <div id="moduleDetails" class="module-details"></div>
        <div id="pipelinePanels" class="pipeline-panels">
          <div class="pipeline-card">
            <h4>Panel 1 · Camera</h4>
            <video id="video" class="camera-video" controls preload="metadata"></video>
          </div>
          <div class="pipeline-card">
            <h4>Panel 2 · VLA 输出</h4>
            <div id="vlaChunkScroll" class="chunk-scroll"><canvas id="vlaChunkCanvas" class="pipeline-canvas chunk-canvas"></canvas></div>
          </div>
          <div class="pipeline-card">
            <h4>Panel 3 · Motion</h4>
            <div id="motionPoseWrap" class="pose-crop target"><div class="pose-empty">等待 MuJoCo 目标姿态渲染</div>
              <video id="motionPoseVideo" muted preload="metadata"></video></div>
          </div>
          <div class="pipeline-card">
            <h4>Panel 4 · Real</h4>
            <div id="realPoseWrap" class="pose-crop actual"><div class="pose-empty">等待 MuJoCo 真机姿态渲染</div>
              <video id="realPoseVideo" muted preload="metadata"></video></div>
          </div>
        </div>
      </div>
      <div class="columns">
        <section>
          <div class="panel"><h3>时间与标注快捷操作</h3>
            <div class="timebar"><input id="seek" type="range" min="0" max="1" step=".02" value="0">
              <span id="timeText" class="time">0.00 / 0.00 s</span></div>
            <div class="quick"><button onclick="setAnomaly(false)">设为异常开始</button>
              <button onclick="setAnomaly(true)">设为异常结束</button>
              <button onclick="addPhaseAtCursor()">从当前时间添加阶段</button></div>
          </div>
          <div class="panel"><h3>MuJoCo G1 + DEX3-1 运动学回放</h3>
            <div class="sim-toolbar">
              <button id="generateMujoco" onclick="generateMujocoRender()">生成本回合运动学回放</button>
              <span id="mujocoStatus" class="meta">正在检查缓存...</span>
            </div>
            <video id="mujocoVideo" class="sim-video" controls muted preload="metadata"></video>
            <div class="legend">
              <span><i class="dot" style="background:#8fe1b2"></i>绿色：真机实测姿态</span>
              <span><i class="dot" style="background:#76aaff"></i>蓝色：WBC 目标姿态</span>
            </div>
            <div class="sim-hint">使用 43 自由度 G1 + DEX3-1 网格离线渲染。这里只按日志重放关节姿态，不执行重力、接触、碰撞或控制器动力学仿真。</div>
          </div>
          <div class="panel"><h3>实际动作链路</h3>
            <div class="action-controls">
              <label>关节<select id="jointSelect" onchange="renderActionCharts()"></select></label>
              <label>动作编码维度<select id="tokenSelect" onchange="renderActionCharts()"></select></label>
            </div>
            <div class="chartbox">
              <div id="jointChartTitle" class="chart-title">关节目标与实际状态</div>
              <div class="legend">
                <span><i class="dot" style="background:#57a0ff"></i>WBC 目标位置</span>
                <span><i class="dot" style="background:#43cb91"></i>实际关节位置</span>
                <span><i class="dot" style="background:#f3b84b"></i>实际关节速度</span>
              </div>
              <canvas id="jointActionChart" class="action-chart"></canvas>
            </div>
            <div class="chartbox" style="margin-top:8px">
              <div id="tokenChartTitle" class="chart-title">SONIC/VLA 动作编码</div>
              <div class="legend"><span><i class="dot" style="background:#b98cff"></i>最终记录的 motion token</span></div>
              <canvas id="tokenActionChart" class="action-chart"></canvas>
            </div>
          </div>
          <div class="panel"><h3>诊断曲线（竖线跟随视频）</h3><div id="charts" class="charts"></div></div>
          <div class="panel"><h3>事件与推理记录</h3><div id="events" class="event-list"></div></div>
        </section>
        <section>
          <div class="panel"><h3>回合标注</h3>
            <div class="formgrid">
              <div class="field"><label>人工结果</label><select id="outcome">
                <option value="success">成功</option><option value="failure">失败</option>
                <option value="discarded">废弃</option><option value="unknown">未知</option></select></div>
              <div class="field"><label>失败阶段</label><select id="failurePhase"></select></div>
              <div class="field"><label>异常开始（秒）</label><input id="anomalyTime" type="number" min="0" step=".02"></div>
              <div class="field"><label>异常结束（秒）</label><input id="anomalyEnd" type="number" min="0" step=".02"></div>
              <div class="field"><label>根因置信度（0-1）</label><input id="confidence" type="number" min="0" max="1" step=".05"></div>
              <div class="field"><label>是否可恢复</label><select id="recoverable">
                <option value="unknown">暂不确定</option><option value="yes">可以恢复</option>
                <option value="no">无法恢复</option></select></div>
              <div class="field full"><label>根因（可多选）</label><div id="causes" class="causes"></div></div>
              <div class="field full"><label>可见失败现象</label><textarea id="symptom"></textarea></div>
              <div class="field full"><label>根因分析</label><textarea id="causeNotes"></textarea></div>
              <div class="field full"><label>建议纠正</label><textarea id="correction"></textarea></div>
              <div class="field"><label>标注人</label><input id="annotator"></div>
              <div class="field"><label>标签（逗号分隔）</label><input id="tags"></div>
              <div class="field full"><label>备注</label><textarea id="notes"></textarea></div>
            </div>
          </div>
          <div class="panel"><h3>任务阶段区间</h3><div id="phaseRows"></div>
            <button class="smallbtn" onclick="addPhase()">+ 添加阶段</button></div>
          <button id="save" class="save" onclick="saveAnnotation()">保存标注</button>
          <div id="status" class="status">标注写入独立目录，不修改原始数据</div>
        </section>
      </div>
    </div>
  </main>
</div>
<script>
const state={meta:null,episodes:[],selected:null,detail:null,annotation:null,camera:null,selectedModule:'camera',dirty:false};
const $=id=>document.getElementById(id);
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const phaseLabel=x=>state.meta?.phase_labels?.[x]||x;
const causeLabel=x=>state.meta?.root_cause_labels?.[x]||x;
const outcomeLabel=x=>state.meta?.outcome_labels?.[x]||x;
const seriesLabel=x=>state.meta?.series_labels?.[x]||x;
const graphLayout={
  camera:{x:35,y:54,w:150,h:70},robot_state:{x:35,y:194,w:150,h:78},prompt:{x:35,y:322,w:150,h:70},
  observation:{x:330,y:50,w:190,h:92},vla:{x:625,y:50,w:180,h:92},scheduler:{x:930,y:50,w:205,h:92},
  sonic:{x:565,y:194,w:220,h:82},
  low_control:{x:930,y:194,w:190,h:82},robot:{x:565,y:322,w:220,h:82}
};
const graphEdgePaths={
  'camera>observation':'M185 89 C245 89 285 96 330 96',
  'robot_state>observation':'M185 233 C260 233 270 122 330 120',
  'prompt>observation':'M185 357 C260 357 265 138 330 136',
  'observation>vla':'M520 96 L625 96',
  'vla>scheduler':'M805 96 L930 96',
  'scheduler>sonic':'M1032 142 C1018 174 845 235 785 235',
  'sonic>low_control':'M785 235 L930 235',
  'low_control>robot':'M1025 276 C960 334 845 370 785 363',
  'robot>robot_state':'M565 363 C405 392 245 306 185 240',
  'camera>robot_state':'M110 124 C110 155 110 175 110 194'
};
const graphEdgeLabels={
  'camera>observation':{x:258,y:82},
  'robot_state>observation':{x:258,y:146},
  'prompt>observation':{x:258,y:170},
  'observation>vla':{x:572,y:96},
  'vla>scheduler':{x:868,y:96},
  'scheduler>sonic':{x:905,y:181},
  'sonic>low_control':{x:858,y:235},
  'low_control>robot':{x:875,y:340},
  'robot>robot_state':{x:350,y:306},
  'camera>robot_state':{x:154,y:158}
};
async function api(url,opts={}){const r=await fetch(url,opts);const data=await r.json();
  if(!r.ok)throw new Error(data.error||r.statusText);return data}
function status(msg,error=false){$('status').textContent=msg;$('status').style.color=error?'var(--danger)':'var(--muted)'}
async function init(){state.meta=await api('/api/episodes');state.episodes=state.meta.episodes;
  $('datasetMeta').textContent=`${state.meta.dataset} · ${state.episodes.length} 个回合 · ${state.meta.fps} 帧/秒 · 标注文件: ${state.meta.annotations}`;
  $('failurePhase').innerHTML=state.meta.phases.map(x=>`<option value="${x}">${phaseLabel(x)}</option>`).join('');
  $('causes').innerHTML=state.meta.root_causes.map(x=>`<label class="cause"><input type="checkbox" value="${x}"> ${causeLabel(x)}</label>`).join('');
  renderList()}
function renderList(){const q=$('search').value.toLowerCase(),filter=$('outcomeFilter').value;
  const rows=state.episodes.filter(ep=>(!filter||ep.recorded_outcome===filter)&&
    (`${ep.episode_index} ${(ep.tasks||[]).join(' ')}`.toLowerCase().includes(q)));
  $('episodeList').innerHTML=rows.map(ep=>`<div class="episode ${state.selected===ep.episode_index?'active':''}" onclick="selectEpisode(${ep.episode_index})">
    <div class="episode-title"><span>回合 ${ep.episode_index}</span><span>${Number(ep.duration_s||0).toFixed(1)} 秒</span></div>
    <div class="meta">${esc((ep.tasks||[]).join(' · '))}</div><div class="badges">
    <span class="badge ${ep.recorded_outcome}">${outcomeLabel(ep.recorded_outcome)}</span>
    ${ep.annotated?'<span class="badge annotated">已标注</span>':''}
    ${ep.failure_phase?`<span class="badge">${phaseLabel(ep.failure_phase)}</span>`:''}</div></div>`).join('')}
async function selectEpisode(idx){if(state.dirty&&!confirm('当前标注尚未保存，确认切换？'))return;
  state.selected=idx;state.dirty=false;renderList();$('empty').style.display='none';$('workspace').classList.add('open');
  $('episodeTitle').textContent=`回合 ${idx}`;status('正在加载回合数据...');
  try{const [detail,annotation]=await Promise.all([api(`/api/episodes/${idx}`),api(`/api/annotations/${idx}`)]);
    state.detail=detail;state.annotation=annotation;const ep=state.episodes.find(x=>x.episode_index===idx);
    $('recordedBadge').textContent=`采集结果: ${outcomeLabel(ep.recorded_outcome)}`;$('recordedBadge').className=`badge ${ep.recorded_outcome}`;
    state.camera=ep.available_cameras[0]||null;renderCameraTabs(ep);loadVideo();fillForm();
    renderActionControls();renderActionCharts();renderArchitecture();renderCharts();renderEvents();loadMujocoRenderStatus();status('已加载')}
  catch(e){status(`加载失败: ${e.message}`,true)}}
function renderCameraTabs(ep){$('cameraTabs').innerHTML=ep.available_cameras.map(cam=>`<button class="${cam===state.camera?'active':''}"
  onclick="switchCamera('${encodeURIComponent(cam)}')">${cam.includes('ego_view')?'主视角':cam.includes('left')?'左腕相机':cam.includes('right')?'右腕相机':esc(cam)}</button>`).join('')}
function switchCamera(encoded){state.camera=decodeURIComponent(encoded);renderCameraTabs(state.episodes.find(x=>x.episode_index===state.selected));loadVideo()}
function loadVideo(){const v=$('video');if(!state.camera){v.removeAttribute('src');return}v.src=`/video/${encodeURIComponent(state.camera)}/${state.selected}.mp4`;v.load()}
function fillForm(){const a=state.annotation;$('outcome').value=a.outcome||'unknown';$('failurePhase').value=a.failure_phase||'unknown';
  $('anomalyTime').value=a.anomaly_time_s??'';$('anomalyEnd').value=a.anomaly_end_time_s??'';
  $('confidence').value=a.root_cause_confidence??'';$('recoverable').value=a.recoverable||'unknown';
  $('symptom').value=a.visible_symptom||'';$('causeNotes').value=a.root_cause_notes||'';
  $('correction').value=a.suggested_correction||'';$('annotator').value=a.annotator||'';
  $('tags').value=(a.tags||[]).join(', ');$('notes').value=a.notes||'';
  document.querySelectorAll('#causes input').forEach(x=>x.checked=(a.root_causes||[]).includes(x.value));
  renderPhases(a.phases||[])}
function renderPhases(rows){$('phaseRows').innerHTML=rows.map((row,i)=>phaseHtml(row,i)).join('')}
function phaseHtml(row,i){return `<div class="phase-row" data-i="${i}"><select>${state.meta.phases.map(x=>`<option value="${x}" ${x===row.phase?'selected':''}>${phaseLabel(x)}</option>`).join('')}</select>
  <input type="number" min="0" step=".02" value="${row.start_s??0}" title="开始秒">
  <input type="number" min="0" step=".02" value="${row.end_s??0}" title="结束秒">
  <input value="${esc(row.note||'')}" placeholder="阶段备注"><button class="remove" onclick="removePhase(${i})">×</button></div>`}
function phaseData(){return [...document.querySelectorAll('.phase-row')].map(row=>{const x=row.querySelectorAll('select,input');
  return{phase:x[0].value,start_s:Number(x[1].value),end_s:Number(x[2].value),note:x[3].value}})}
function addPhase(start=null){const rows=phaseData(),t=start??Number($('video').currentTime||0);rows.push({phase:'unknown',start_s:t,end_s:t,note:''});renderPhases(rows);markDirty()}
function addPhaseAtCursor(){addPhase(Number($('video').currentTime||0))}
function removePhase(i){const rows=phaseData();rows.splice(i,1);renderPhases(rows);markDirty()}
function setAnomaly(end){const t=Number($('video').currentTime||0).toFixed(2);$(end?'anomalyEnd':'anomalyTime').value=t;markDirty()}
function markDirty(){state.dirty=true;status('有未保存修改')}
function nearestIndex(times,t){if(!times||!times.length)return-1;let best=0,bestDelta=Infinity;
  times.forEach((value,i)=>{const d=Math.abs(Number(value)-t);if(d<bestDelta){best=i;bestDelta=d}});return best}
function nearestByTime(rows,t){if(!rows||!rows.length)return null;let best=rows[0],bestDelta=Infinity;
  rows.forEach(row=>{const d=Math.abs(Number(row.episode_time_s||0)-t);if(d<bestDelta){best=row;bestDelta=d}});return best}
function fmtNumber(value,unit=''){const n=Number(value);if(!Number.isFinite(n))return '暂无';
  const digits=Math.abs(n)>=100?1:Math.abs(n)>=10?2:3;return `${n.toFixed(digits)}${unit?` ${unit}`:''}`}
function signalLabel(name){return seriesLabel(name).replace(/（.*?）/g,'')||name}
function currentSample(t=null){if(!state.detail)return{index:-1,time:null,columns:{}};const ts=state.detail.timeseries;
  const cursor=t==null?Number($('video').currentTime||0):t,index=nearestIndex(ts.time_s,cursor),columns={};
  Object.entries(ts.sample_columns||{}).forEach(([key,values])=>columns[key]=values[index]);
  return{index,time:ts.time_s?.[index],columns}}
function signalValue(name,t=null){if(!state.detail)return null;const cursor=t==null?Number($('video').currentTime||0):t,ts=state.detail.timeseries;
  const sample=currentSample(cursor),idx=sample.index,actions=ts.actions||{};
  if(name==='flow')return 'flow';
  if(name==='task_prompt'){const ep=state.episodes.find(x=>x.episode_index===state.selected);return(ep?.tasks||[]).join(' · ')||'暂无'}
  if((ts.series||{})[name])return(ts.series[name]||[])[idx];
  if((ts.sample_columns||{})[name])return(ts.sample_columns[name]||[])[idx];
  if(name==='policy_total_ms'){const row=nearestByTime((state.detail.inference_events||[]).filter(x=>x.event==='inference_chunk'),cursor);return row?.inference_ms}
  if(name==='inference_event_count'){const s=state.detail.inference_summary||{};return `${s.chunk_count||0} chunk / ${s.error_count||0} 异常`}
  if(['publish_frame_index','chunk_id','chunk_index','blend_alpha'].includes(name)){
    const row=nearestByTime(state.detail.published_actions||[],cursor);
    const map={publish_frame_index:'frame_index',chunk_id:'chunk_id',chunk_index:'chunk_index',blend_alpha:'blend_alpha'};
    return row?.[map[name]]}
  if(name==='action.wbc'){const joint=Number($('jointSelect').value||0);
    const target=matrixColumn(actions.wbc_target,joint)[idx],actual=matrixColumn(actions.joint_state,joint)[idx];
    return `target ${fmtNumber(target,'rad')} / actual ${fmtNumber(actual,'rad')}`}
  if(name==='action.motion_token'){const token=Number($('tokenSelect').value||0),row=(actions.motion_token||[])[idx]||[];
    const norm=Math.sqrt(row.reduce((sum,x)=>sum+(Number.isFinite(Number(x))?Number(x)*Number(x):0),0));
    return `dim ${token}: ${fmtNumber(row[token])} / norm ${fmtNumber(norm)}`}
  return null}
function edgeClass(edge,value){if(edge.metric==='flow')return'ok';const n=Number(value);if(!Number.isFinite(n))return'missing';
  if(edge.metric==='tracking_rmse_rad')return n>.35?'bad':n>.18?'warn':'ok';
  if(edge.metric==='motor_error_count')return n>0?'bad':'ok';
  if(edge.unit==='ms'){const a=Math.abs(n);return a>250?'bad':a>100?'warn':'ok'}
  return'ok'}
function renderLatencyStrip(t=null){if(!state.detail||!state.meta)return;const cursor=t==null?Number($('video').currentTime||0):t;
  $('latencyStrip').innerHTML=(state.meta.architecture_edges||[]).filter(edge=>edge.metric!=='flow').map(edge=>{
    const value=signalValue(edge.metric,cursor),klass=edgeClass(edge,value);
    return`<div class="latency-chip ${klass}"><strong>${esc(edge.label)}</strong><span>${fmtNumber(value,edge.unit)}</span></div>`}).join('')}
function edgeLabel(edge,value){if(edge.metric==='task_prompt')return edge.label;
  if(edge.metric==='flow')return edge.label;
  if(edge.metric==='chunk_id')return `chunk ${value??'暂无'}`;
  if(['publish_frame_index','chunk_id','chunk_index','blend_alpha'].includes(edge.metric))return `${value??'暂无'}`;
  if(typeof value==='string')return edge.label;
  return fmtNumber(value,edge.unit)}
function pathMidpoint(path){const nums=[...path.matchAll(/-?\d+(?:\.\d+)?/g)].map(x=>Number(x[0]));
  const points=[];for(let i=0;i<nums.length-1;i+=2)points.push({x:nums[i],y:nums[i+1]});
  return points[Math.floor(points.length/2)]||{x:600,y:250}}
function renderGraphEdges(t=null){if(!$('archEdges')||!state.meta)return;const cursor=t==null?Number($('video').currentTime||0):t;
  const paths=(state.meta.architecture_edges||[]).filter(edge=>edge.show_in_graph!==false).map((edge,i)=>{
    const key=`${edge.from}>${edge.to}`,path=graphEdgePaths[key]||'',graphEdge={...edge,label:edge.graph_label??edge.label,metric:edge.graph_metric??edge.metric,unit:edge.graph_unit??edge.unit};
    const value=signalValue(graphEdge.metric,cursor),klass=edgeClass(graphEdge,value);
    const mid=graphEdgeLabels[key]||pathMidpoint(path),text=edgeLabel(graphEdge,value),width=Math.min(150,Math.max(58,text.length*7+14));
    const label=edge.show_label===false?'':`<g class="edge-label ${klass}"><rect x="${mid.x-width/2}" y="${mid.y-12}" width="${width}" height="22"></rect>
      <text x="${mid.x}" y="${mid.y+4}" text-anchor="middle">${esc(text.length>20?text.slice(0,19)+'…':text)}</text></g>`;
    return`<path class="edge-path ${edge.kind||'main'}" d="${path}" marker-end="url(#arrow-${edge.kind||'main'})"></path>
      ${label}`}).join('');
  $('archEdges').innerHTML=`<defs>
    <marker id="arrow-main" markerWidth="10" markerHeight="10" refX="8.5" refY="5" orient="auto"><path d="M1,1 L9,5 L1,9 Z" fill="#aeb9c8"/></marker>
    <marker id="arrow-soft" markerWidth="10" markerHeight="10" refX="8.5" refY="5" orient="auto"><path d="M1,1 L9,5 L1,9 Z" fill="#768696"/></marker>
    <marker id="arrow-feedback" markerWidth="10" markerHeight="10" refX="8.5" refY="5" orient="auto"><path d="M1,1 L9,5 L1,9 Z" fill="#b98cff"/></marker>
    <marker id="arrow-log" markerWidth="10" markerHeight="10" refX="8.5" refY="5" orient="auto"><path d="M1,1 L9,5 L1,9 Z" fill="#7d8997"/></marker>
  </defs>${paths}`}
function renderCursorContext(t=null){if(!$('cursorContext')||!state.detail)return;const cursor=t==null?Number($('video').currentTime||0):t;
  const sample=currentSample(cursor);
  $('cursorContext').innerHTML=[
    ['视频时间',fmtNumber(cursor,'s')],
    ['最近样本',sample.time==null?'暂无':`${fmtNumber(sample.time,'s')} · sample ${sample.index}`],
    ['episode frame',sample.columns.frame_index??'暂无'],
    ['dataset index',sample.columns.index??'暂无']
  ].map(([k,v])=>`<span>${esc(k)} <strong>${esc(v)}</strong></span>`).join('')}
function renderArchitecture(){if(!state.meta)return;renderLatencyStrip();const modules=state.meta.architecture_modules||[];
  $('archMap').innerHTML=`<div class="arch-stage">
    <div class="arch-lane-band" style="top:18px;height:140px"></div><div class="arch-lane-label" style="top:30px">实时输入与高层决策</div>
    <div class="arch-lane-band" style="top:176px;height:118px"></div><div class="arch-lane-label" style="top:188px">Token 到控制动作</div>
    <div class="arch-lane-band" style="top:306px;height:110px"></div><div class="arch-lane-label" style="top:318px">真机执行与反馈</div>
    <svg id="archEdges" class="arch-svg" viewBox="0 0 1180 430" aria-hidden="true"></svg>
    ${modules.map(m=>{const p=graphLayout[m.id]||{x:20,y:20,w:160,h:70};return`<button class="arch-node ${m.kind} ${m.id===state.selectedModule?'active':''}" style="left:${p.x}px;top:${p.y}px;width:${p.w}px;height:${p.h}px" onclick="selectModule('${m.id}')">
      <div class="arch-title">${esc(m.label)}</div><div class="arch-sub">${esc(m.subtitle)}</div></button>`}).join('')}
    <div class="arch-legend"><span><i class="legend-line"></i>控制/数据主链</span><span><i class="legend-line feedback"></i>真机反馈</span></div>
  </div>`;
  renderGraphEdges();renderCursorContext();renderModuleDetails();renderPipelinePanels()}
function selectModule(id){state.selectedModule=id;renderArchitecture()}
function renderModuleDetails(t=null){if(!state.detail||!state.meta)return;const cursor=t==null?Number($('video').currentTime||0):t;
  const module=(state.meta.architecture_modules||[]).find(x=>x.id===state.selectedModule)||state.meta.architecture_modules?.[0];
  if(!module)return;const pub=nearestByTime(state.detail.published_actions||[],cursor),rows=[];
  if(['scheduler','sonic'].includes(module.id)&&pub)rows.push(['最近发布动作',`t=${fmtNumber(pub.episode_time_s,'s')} · chunk ${pub.chunk_id ?? '暂无'} / step ${pub.chunk_index ?? '暂无'} / frame ${pub.frame_index ?? '暂无'}`]);
  for(const name of module.signals||[]){const value=signalValue(name,cursor);rows.push([signalLabel(name),typeof value==='number'?fmtNumber(value,(state.meta.architecture_edges||[]).find(e=>e.metric===name)?.unit||''):value??'暂无'])}
  $('moduleDetails').innerHTML=`<div class="module-card"><h4>${esc(module.label)} · 模块数据</h4><div class="kv">
    ${rows.map(([k,v])=>`<div>${esc(k)}</div><div>${esc(v)}</div>`).join('')||'<div>暂无专属字段</div><div>可从图上边标签和诊断曲线查看</div>'}</div></div>
    <div class="module-card"><h4>证据与缺口</h4>
      <div class="tagrow">${(module.evidence||[]).map(x=>`<span class="tag">${esc(x)}</span>`).join('')}</div>
      <div style="height:8px"></div>
      <div class="tagrow">${(module.missing||[]).map(x=>`<span class="tag missing">${esc(x)}</span>`).join('')}</div>
    </div>`}
function canvas2d(canvas){const ratio=devicePixelRatio||1,w=canvas.clientWidth||300,h=canvas.clientHeight||110;
  canvas.width=w*ratio;canvas.height=h*ratio;const c=canvas.getContext('2d');c.setTransform(ratio,0,0,ratio,0,0);
  c.clearRect(0,0,w,h);return{c,w,h}}
function drawEmptyCanvas(canvas,text){if(!canvas)return;const {c,w,h}=canvas2d(canvas);
  c.fillStyle='#0a0d13';c.fillRect(0,0,w,h);c.fillStyle='#97a3b6';c.font='12px sans-serif';c.textAlign='center';c.fillText(text,w/2,h/2)}
function renderKv(id,pairs){const el=$(id);if(!el)return;el.innerHTML=pairs.map(([k,v])=>`<div>${esc(k)}</div><div>${esc(v)}</div>`).join('')}
function rowAt(matrix,index){const row=(matrix||[])[index];return Array.isArray(row)?row:[]}
function vectorNorm(row){return Math.sqrt((row||[]).reduce((sum,x)=>{const n=Number(x);return sum+(Number.isFinite(n)?n*n:0)},0))}
function drawPoseBars(canvas,series){if(!canvas){return}const values=series.flatMap(s=>(s.values||[]).map(Number).filter(Number.isFinite));
  if(!values.length){drawEmptyCanvas(canvas,'暂无姿态数据');return}const {c,w,h}=canvas2d(canvas),count=Math.min(18,Math.max(...series.map(s=>(s.values||[]).length)));
  const max=Math.max(.001,...values.map(v=>Math.abs(v))),gap=3,barW=(w-16-(count-1)*gap)/count,zero=h/2;
  c.fillStyle='#0a0d13';c.fillRect(0,0,w,h);c.strokeStyle='#303a49';c.beginPath();c.moveTo(8,zero);c.lineTo(w-8,zero);c.stroke();
  series.forEach((s,si)=>{c.fillStyle=s.color;(s.values||[]).slice(0,count).forEach((value,i)=>{
      const n=Number(value);if(!Number.isFinite(n))return;const x=8+i*(barW+gap)+si*Math.max(2,barW*.18),bh=Math.abs(n)/max*(h*.42);
      c.fillRect(x,n>=0?zero-bh:zero,Math.max(2,barW*.42),bh)})});
  c.fillStyle='#97a3b6';c.font='10px sans-serif';c.textAlign='left';c.fillText(`前 ${count} 个关节`,8,12);c.textAlign='right';c.fillText(`±${max.toFixed(2)} rad`,w-8,12)}
function drawChunkTimeline(canvas,rows,events,cursor){if(!canvas)return;if(!rows||!rows.length){drawEmptyCanvas(canvas,'暂无 VLA chunk 记录');return}
  const current=nearestByTime(rows,cursor),byChunk=new Map(),startByChunk=new Map();
  rows.forEach(row=>{if(row.chunk_id==null)return;const id=Number(row.chunk_id);
    if(!byChunk.has(id))byChunk.set(id,[]);byChunk.get(id).push(row)});
  (events||[]).filter(e=>e.event==='inference_chunk'&&e.chunk_id!=null).forEach(e=>{
    const start=Number(e.selected_start_index);if(Number.isFinite(start))startByChunk.set(Number(e.chunk_id),Math.max(0,Math.min(39,start)))});
  const chunkIds=[...new Set([...byChunk.keys(),...startByChunk.keys()])].sort((a,b)=>a-b),currentId=Number(current?.chunk_id);
  const currentPos=Math.max(0,chunkIds.indexOf(currentId));
  const maxStep=39,stepW=14,labelW=74,right=20,rowH=26,baseY=28,offsetById=new Map();
  let offset=0;chunkIds.forEach((id,i)=>{if(i>0){const prevId=chunkIds[i-1],prevRows=byChunk.get(prevId)||[];
      const prevEnd=prevRows.length?Math.max(...prevRows.map(r=>Number(r.chunk_index)||0)):maxStep;
      const selectedStart=startByChunk.get(id)??0;offset+=Math.max(1,prevEnd-selectedStart)*stepW}
    offsetById.set(id,offset)});
  const currentOffset=offsetById.get(currentId)??0,currentBase=labelW+Math.max(220,(canvas.parentElement?.clientWidth||420)*.45);
  const virtualWidth=Math.ceil(Math.max((canvas.parentElement?.clientWidth||420)*2.2,currentBase+maxStep*stepW+right+240));
  const virtualHeight=Math.max(canvas.parentElement?.clientHeight||390,baseY+chunkIds.length*rowH+24);
  canvas.style.width=`${virtualWidth}px`;canvas.style.height=`${virtualHeight}px`;
  const {c,w,h}=canvas2d(canvas);c.fillStyle='#0a0d13';c.fillRect(0,0,w,h);c.font='11px sans-serif';
  chunkIds.forEach((id,i)=>{const chunkRows=(byChunk.get(id)||[]).sort((a,b)=>(Number(a.chunk_index)||0)-(Number(b.chunk_index)||0));
    const y=baseY+i*rowH,steps=chunkRows.map(r=>Number(r.chunk_index)).filter(Number.isFinite);
    const selectedStart=startByChunk.get(id)??(steps.length?Math.max(0,Math.min(...steps)):0);
    const usedMax=steps.length?Math.max(...steps):selectedStart,isCurrent=id===currentId;
    const rowLeft=currentBase+(offsetById.get(id)||0)-currentOffset,stepX=step=>rowLeft+Math.max(0,Math.min(maxStep,step))*stepW;
    if(rowLeft>w+80||rowLeft+maxStep*stepW<-80)return;
    c.fillStyle=isCurrent?'#e8edf5':'#97a3b6';c.textAlign='right';c.fillText(`chunk ${id}`,rowLeft-10,y+4);
    c.strokeStyle='#253040';c.lineWidth=7;c.beginPath();c.moveTo(rowLeft,y);c.lineTo(rowLeft+maxStep*stepW,y);c.stroke();
    if(selectedStart>0){c.strokeStyle=isCurrent?'rgba(87,160,255,.20)':'rgba(151,163,182,.16)';
      c.beginPath();c.moveTo(stepX(0),y);c.lineTo(stepX(selectedStart),y);c.stroke()}
    c.strokeStyle=isCurrent?'#57a0ff':'#617089';c.lineWidth=7;c.beginPath();c.moveTo(stepX(selectedStart),y);c.lineTo(stepX(usedMax),y);c.stroke();
    for(let step=0;step<=maxStep;step+=1){const x=stepX(step),used=steps.includes(step),discarded=step<selectedStart;
      c.fillStyle=used?(isCurrent?'#d8e9ff':'#9aa8bb'):(discarded?'rgba(151,163,182,.28)':'#303a49');
      c.fillRect(x-1,used?y-6:y-3,2,used?12:6)}
    if(selectedStart>0){c.fillStyle=isCurrent?'rgba(87,160,255,.75)':'rgba(151,163,182,.55)';
      c.textAlign='left';c.fillText(`discard 0-${selectedStart-1}`,stepX(0),y-8)}
    if(isCurrent&&current){const step=Number(current.chunk_index)||0,x=stepX(step);
      c.strokeStyle='#ffffff';c.lineWidth=2;c.beginPath();c.moveTo(x,y-11);c.lineTo(x,y+11);c.stroke();
      c.fillStyle='#ffffff';c.textAlign='left';c.fillText(`step ${step}`,Math.min(w-58,x+7),y-8)}
  });
  const currentStep=Number(current?.chunk_index)||0,scroller=canvas.parentElement;
  if(scroller&&Number.isFinite(currentBase)){const targetX=currentBase+currentStep*stepW-scroller.clientWidth*.55;
    const next=Math.max(0,targetX),delta=next-scroller.scrollLeft;
    if(Math.abs(delta)>1)scroller.scrollLeft+=delta*.18}
  if(scroller){const rowIndex=currentPos;
    if(rowIndex>=0){const targetY=baseY+rowIndex*rowH-scroller.clientHeight*.45,deltaY=Math.max(0,targetY)-scroller.scrollTop;
      if(Math.abs(deltaY)>1)scroller.scrollTop+=deltaY*.18}}
  c.fillStyle='#97a3b6';c.textAlign='left';c.font='10px sans-serif';c.fillText('old',labelW,12);
  c.textAlign='right';c.fillText(`step ${maxStep}`,w-right,12)}
function renderPipelinePanels(t=null){if(!state.detail)return;const cursor=t==null?Number($('video').currentTime||0):t;
  drawChunkTimeline($('vlaChunkCanvas'),state.detail.published_actions||[],state.detail.inference_events||[],cursor)}
let pipelineFrameHandle=null,pipelineFrameHandleType='';
function cancelPipelineLoop(){const v=$('video');if(pipelineFrameHandle==null)return;
  if(pipelineFrameHandleType==='video'&&v.cancelVideoFrameCallback)v.cancelVideoFrameCallback(pipelineFrameHandle);
  else cancelAnimationFrame(pipelineFrameHandle);pipelineFrameHandle=null;pipelineFrameHandleType=''}
function schedulePipelineLoop(){const v=$('video');if(pipelineFrameHandle!=null||!v||v.paused||v.ended)return;
  const tick=()=>{pipelineFrameHandle=null;const t=Number(v.currentTime||0);renderPipelinePanels(t);syncMujocoTime(false);
    if(!v.paused&&!v.ended)schedulePipelineLoop()};
  if(v.requestVideoFrameCallback){pipelineFrameHandleType='video';pipelineFrameHandle=v.requestVideoFrameCallback(tick)}
  else{pipelineFrameHandleType='raf';pipelineFrameHandle=requestAnimationFrame(tick)}}
function renderEvents(){const out=[];for(const e of state.detail.events||[]){const base=state.detail.recorded_outcome.started_at_unix_s;
  const eventNames={collector_started:'采集器启动',keyboard:'键盘操作',episode_started:'回合开始',
    episode_stop_requested:'请求结束回合',episode_saved:'回合保存完成',motion_changed:'参考动作切换'};
  const t=base?Number(e.time_unix_s-base):0;out.push({t,text:`${eventNames[e.event]||e.event}${e.key?'，按键 '+e.key:''}`})}
  for(const e of state.detail.inference_events||[]){const inferNames={inference_chunk:'模型推理',
    inference_error:'模型推理异常',action_bound_violation:'动作超出范围',worker_exception:'推理线程异常'};
    out.push({t:Number(e.episode_time_s),text:`${inferNames[e.event]||e.event}${e.inference_ms!=null?'，耗时 '+Number(e.inference_ms).toFixed(1)+' 毫秒':''}${e.selected_start_index!=null?'，跳过 '+e.selected_start_index+' 帧':''}`})}
  out.sort((a,b)=>a.t-b.t);$('events').innerHTML=out.map(e=>`<div class="event"><button onclick="seekTo(${e.t})">${e.t.toFixed(2)}s</button>${esc(e.text)}</div>`).join('')||'<span class="meta">无事件</span>'}
function seekTo(t){$('video').currentTime=Math.max(0,t)}
const chartColors=['#57a0ff','#43cb91','#f3b84b','#ff6675','#b98cff','#55d7d1','#f18bd1','#d3dc66','#ef9858','#9da9bc'];
function renderActionControls(){const actions=state.detail.timeseries.actions||{},joint=$('jointSelect'),token=$('tokenSelect');
  const oldJoint=joint.value,oldToken=token.value;
  joint.innerHTML=(actions.joint_names||[]).map((name,i)=>`<option value="${i}">${i} · ${esc(actions.joint_labels[i]||name)}</option>`).join('');
  token.innerHTML=Array.from({length:actions.motion_token_width||0},(_,i)=>`<option value="${i}">维度 ${i}</option>`).join('');
  if(oldJoint&&Number(oldJoint)<(actions.joint_names||[]).length)joint.value=oldJoint;
  else joint.value=String(Math.min(15,Math.max(0,(actions.joint_names||[]).length-1)));
  if(oldToken&&Number(oldToken)<(actions.motion_token_width||0))token.value=oldToken;else token.value='0'}
function matrixColumn(matrix,index){return (matrix||[]).map(row=>Array.isArray(row)?row[index]:null)}
function renderActionCharts(cursor=null){if(!state.detail)return;const ts=state.detail.timeseries,actions=ts.actions||{};
  const jointIndex=Number($('jointSelect').value||0),tokenIndex=Number($('tokenSelect').value||0);
  const jointName=actions.joint_labels?.[jointIndex]||actions.joint_names?.[jointIndex]||`关节 ${jointIndex}`;
  $('jointChartTitle').textContent=`${jointName}：目标、实际位置与速度`;
  $('tokenChartTitle').textContent=`SONIC/VLA 动作编码：维度 ${tokenIndex}`;
  const t=cursor==null?Number($('video').currentTime||0):cursor;
  drawMultiChart($('jointActionChart'),ts.time_s,[
    {values:matrixColumn(actions.wbc_target,jointIndex),color:'#57a0ff'},
    {values:matrixColumn(actions.joint_state,jointIndex),color:'#43cb91'},
    {values:matrixColumn(actions.joint_velocity,jointIndex),color:'#f3b84b'}],t);
  drawMultiChart($('tokenActionChart'),ts.time_s,[
    {values:matrixColumn(actions.motion_token,tokenIndex),color:'#b98cff'}],t);
  renderModuleDetails(t);renderPipelinePanels(t)}
function posePanelVideos(){return[$('motionPoseVideo'),$('realPoseVideo')].filter(Boolean)}
function setPosePanelSource(url=''){for(const video of posePanelVideos()){video.pause();
    if(url){video.src=url;video.load();video.currentTime=Number($('video').currentTime||0)}
    else{video.removeAttribute('src');video.load()}}
  document.querySelectorAll('.pose-empty').forEach(el=>el.style.display=url?'none':'grid')}
async function loadMujocoRenderStatus(){const video=$('mujocoVideo'),button=$('generateMujoco');
  video.pause();video.removeAttribute('src');video.load();setPosePanelSource();button.disabled=false;$('mujocoStatus').textContent='正在检查缓存...';
  try{const result=await api(`/api/render/${state.selected}`);if(result.ready){
      const url=`${result.video_url}?v=${Date.now()}`;video.src=url;video.load();setPosePanelSource(url);button.textContent='重新生成运动学回放';
      $('mujocoStatus').textContent='运动学回放已就绪'}else{button.textContent='生成本回合运动学回放';
      $('mujocoStatus').textContent='尚未生成，首次生成可能需要几十秒'}}
  catch(e){$('mujocoStatus').textContent=`检查失败：${e.message}`}}
async function generateMujocoRender(){const button=$('generateMujoco');button.disabled=true;
  $('mujocoStatus').textContent='正在用 MuJoCo 渲染，请稍候...';
  try{const result=await api(`/api/render/${state.selected}`,{method:'POST'});
    const video=$('mujocoVideo'),url=`${result.video_url}?v=${Date.now()}`;video.src=url;video.load();setPosePanelSource(url);
    video.currentTime=Number($('video').currentTime||0);button.textContent='重新生成运动学回放';
    $('mujocoStatus').textContent=result.cached?'已加载缓存':'渲染完成'}
  catch(e){$('mujocoStatus').textContent=`渲染失败：${e.message}`}
  finally{button.disabled=false}}
function syncMujocoTime(force=false){const source=$('video'),simulation=$('mujocoVideo');
  const videos=[simulation,...posePanelVideos()].filter(v=>v?.src);
  for(const v of videos){if(!Number.isFinite(v.duration))continue;
    const drift=Number(source.currentTime||0)-Number(v.currentTime||0),baseRate=Number(source.playbackRate||1);
    if(force||Math.abs(drift)>.18){v.currentTime=source.currentTime;v.playbackRate=baseRate}
    else if(!source.paused){v.playbackRate=Math.max(.85,Math.min(1.15,baseRate+drift*.25))}
    else v.playbackRate=baseRate}}
function drawMultiChart(canvas,times,lines,cursor){const ratio=devicePixelRatio||1,w=canvas.clientWidth||700,h=canvas.clientHeight||180;
  canvas.width=w*ratio;canvas.height=h*ratio;const c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,w,h);
  const vals=lines.flatMap(line=>(line.values||[]).filter(x=>x!=null&&Number.isFinite(x)));if(!vals.length)return;
  let lo=Math.min(...vals),hi=Math.max(...vals);if(lo===hi){lo-=1;hi+=1}const tmax=Math.max(...times,1);
  c.strokeStyle='#303a49';c.lineWidth=1;c.beginPath();c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();
  for(const line of lines){c.strokeStyle=line.color;c.lineWidth=1.4;c.beginPath();let started=false;
    (line.values||[]).forEach((v,i)=>{if(v==null||!Number.isFinite(v))return;const x=times[i]/tmax*w,y=h-7-(v-lo)/(hi-lo)*(h-14);
      if(!started){c.moveTo(x,y);started=true}else c.lineTo(x,y)});c.stroke()}
  c.strokeStyle='#ffffffbb';c.lineWidth=1;c.beginPath();const cursorX=Math.min(cursor/tmax*w,w);c.moveTo(cursorX,0);c.lineTo(cursorX,h);c.stroke();
  c.fillStyle='#97a3b6';c.font='10px sans-serif';c.fillText(hi.toFixed(3),3,10);c.fillText(lo.toFixed(3),3,h-2)}
function renderCharts(){const host=$('charts'),ts=state.detail.timeseries;host.innerHTML='';Object.entries(ts.series||{}).forEach(([name,values],i)=>{
  const box=document.createElement('div');box.className='chartbox';box.innerHTML=`<div class="chart-title">${esc(seriesLabel(name))}</div><canvas></canvas>`;host.appendChild(box);
  box._chart={times:ts.time_s,values,color:chartColors[i%chartColors.length]};drawChart(box.querySelector('canvas'),box._chart,0)})}
function drawChart(canvas,data,cursor){const ratio=devicePixelRatio||1,w=canvas.clientWidth||500,h=canvas.clientHeight||90;canvas.width=w*ratio;canvas.height=h*ratio;
  const c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,w,h);const vals=data.values.filter(x=>x!=null&&Number.isFinite(x));
  if(!vals.length)return;let lo=Math.min(...vals),hi=Math.max(...vals);if(lo===hi){lo-=1;hi+=1}const tmax=Math.max(...data.times,1);
  c.strokeStyle=data.color;c.lineWidth=1.3;c.beginPath();let started=false;data.values.forEach((v,i)=>{if(v==null||!Number.isFinite(v))return;
    const x=data.times[i]/tmax*w,y=h-4-(v-lo)/(hi-lo)*(h-8);if(!started){c.moveTo(x,y);started=true}else c.lineTo(x,y)});c.stroke();
  c.strokeStyle='#ffffffaa';c.lineWidth=1;c.beginPath();c.moveTo(Math.min(cursor/tmax*w,w),0);c.lineTo(Math.min(cursor/tmax*w,w),h);c.stroke();
  c.fillStyle='#97a3b6';c.font='10px sans-serif';c.fillText(hi.toFixed(2),3,10);c.fillText(lo.toFixed(2),3,h-2)}
function updateCursor(){const v=$('video'),t=Number(v.currentTime||0),d=Number(v.duration||state.annotation?.episode_duration_s||0);
  $('seek').max=d||1;$('seek').value=t;$('timeText').textContent=`${t.toFixed(2)} / ${d.toFixed(2)} s`;
  document.querySelectorAll('.chartbox').forEach(box=>{if(box._chart)drawChart(box.querySelector('canvas'),box._chart,t)});
  renderActionCharts(t);renderLatencyStrip(t);renderGraphEdges(t);renderCursorContext(t);syncMujocoTime(false)}
function formPayload(){return{outcome:$('outcome').value,failure_phase:$('failurePhase').value,
  anomaly_time_s:$('anomalyTime').value===''?null:Number($('anomalyTime').value),
  anomaly_end_time_s:$('anomalyEnd').value===''?null:Number($('anomalyEnd').value),
  root_cause_confidence:$('confidence').value===''?null:Number($('confidence').value),
  recoverable:$('recoverable').value,root_causes:[...document.querySelectorAll('#causes input:checked')].map(x=>x.value),
  visible_symptom:$('symptom').value,root_cause_notes:$('causeNotes').value,suggested_correction:$('correction').value,
  annotator:$('annotator').value,tags:$('tags').value.split(',').map(x=>x.trim()).filter(Boolean),notes:$('notes').value,phases:phaseData()}}
async function saveAnnotation(){const b=$('save');b.disabled=true;status('保存中...');
  try{state.annotation=await api(`/api/annotations/${state.selected}`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(formPayload())});
    state.dirty=false;state.meta=await api('/api/episodes');state.episodes=state.meta.episodes;renderList();status(`已保存 ${state.annotation.updated_at}`)}
  catch(e){status(`保存失败: ${e.message}`,true)}finally{b.disabled=false}}
$('search').addEventListener('input',renderList);$('outcomeFilter').addEventListener('change',renderList);
$('video').addEventListener('timeupdate',updateCursor);$('video').addEventListener('loadedmetadata',()=>{updateCursor();renderPipelinePanels()});
$('video').addEventListener('play',()=>{const simulation=$('mujocoVideo');syncMujocoTime(true);
  for(const v of [simulation,...posePanelVideos()]){v.playbackRate=$('video').playbackRate;if(v.src)v.play().catch(()=>{})}
  schedulePipelineLoop()});
$('video').addEventListener('pause',()=>{cancelPipelineLoop();renderPipelinePanels();[$('mujocoVideo'),...posePanelVideos()].forEach(v=>v.pause())});
$('video').addEventListener('ratechange',()=>[$('mujocoVideo'),...posePanelVideos()].forEach(v=>v.playbackRate=$('video').playbackRate));
$('video').addEventListener('seeking',()=>{syncMujocoTime(true);renderPipelinePanels()});
$('video').addEventListener('seeked',()=>{syncMujocoTime(true);renderPipelinePanels()});
$('seek').addEventListener('input',e=>{$('video').currentTime=Number(e.target.value);syncMujocoTime(true);renderPipelinePanels()});
$('mujocoVideo').addEventListener('seeking',()=>{if(Math.abs($('video').currentTime-$('mujocoVideo').currentTime)>.15)
  $('video').currentTime=$('mujocoVideo').currentTime});
document.addEventListener('input',e=>{const viewOnly=['seek','jointSelect','tokenSelect'];
  if($('workspace').contains(e.target)&&!viewOnly.includes(e.target.id))markDirty()});
window.addEventListener('beforeunload',e=>{if(state.dirty){e.preventDefault();e.returnValue=''}});
init().catch(e=>$('datasetMeta').textContent=`加载失败: ${e.message}`);
</script>
</body></html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="LeRobot dataset root")
    parser.add_argument(
        "--annotations-dir",
        type=Path,
        default=None,
        help="Independent annotation output directory (default: annotations/<run>)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.dataset.resolve()
    if not (root / "meta" / "info.json").exists():
        raise SystemExit(f"Not a LeRobot dataset: missing {root / 'meta/info.json'}")

    annotations_dir = (
        args.annotations_dir.resolve()
        if args.annotations_dir
        else REPO_ROOT / "annotations" / root.name
    )
    annotations_path = annotations_dir / "failure_annotations.jsonl"
    history_path = annotations_dir / "failure_annotations_history.jsonl"
    render_dir = annotations_dir / "mujoco_renders"

    AnnotationHandler.dataset_root = root
    AnnotationHandler.annotations_path = annotations_path
    AnnotationHandler.history_path = history_path
    AnnotationHandler.render_dir = render_dir

    class Server(ThreadingHTTPServer):
        daemon_threads = True

    print(f"[failure-annotation] dataset (read-only): {root}")
    print(f"[failure-annotation] annotations: {annotations_path}")
    print(f"[failure-annotation] open http://{args.host}:{args.port}/")
    with Server((args.host, args.port), AnnotationHandler) as server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n[failure-annotation] shutting down")


if __name__ == "__main__":
    main()
