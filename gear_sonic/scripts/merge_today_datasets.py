"""Merge today's LeRobot-format autopilot datasets into a single dataset.

Reads from outputs/2026-05-26-* (skipping the empty 11-09-05 run) and writes
a concatenated dataset to outputs/2026-05-26-merged/ with re-numbered episode
and frame indices. Schema, fps, and the single task are identical across
sources, so we only rewrite the index columns and the meta files.
"""

import json
import shutil
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

BASE = Path("/home/jg/baseline/GR00T-WholeBodyControl/outputs")
SOURCES = [
    "2026-05-26-09-51-07",
    "2026-05-26-10-34-05",
    "2026-05-26-10-51-07",
    "2026-05-26-11-10-54",
    "2026-05-26-11-24-15",
]
DEST = BASE / "2026-05-26-merged"
VIDEO_KEY = "observation.images.ego_view"


def main() -> None:
    if DEST.exists():
        raise SystemExit(f"Destination already exists: {DEST}")

    (DEST / "data" / "chunk-000").mkdir(parents=True)
    (DEST / "videos" / "chunk-000" / VIDEO_KEY).mkdir(parents=True)
    (DEST / "meta").mkdir(parents=True)

    new_ep_index = 0
    global_frame_offset = 0
    merged_episodes: list[dict] = []
    merged_stats: list[dict] = []
    total_frames = 0

    for src_name in SOURCES:
        src = BASE / src_name
        with open(src / "meta" / "episodes.jsonl") as f:
            src_eps = [json.loads(line) for line in f if line.strip()]
        with open(src / "meta" / "episodes_stats.jsonl") as f:
            src_stats = {
                json.loads(line)["episode_index"]: json.loads(line)
                for line in f
                if line.strip()
            }

        for ep in src_eps:
            old_idx = ep["episode_index"]
            src_parquet = src / "data" / "chunk-000" / f"episode_{old_idx:06d}.parquet"
            src_video = (
                src / "videos" / "chunk-000" / VIDEO_KEY / f"episode_{old_idx:06d}.mp4"
            )
            if not src_parquet.exists() or not src_video.exists():
                raise FileNotFoundError(
                    f"Missing files for {src_name} ep {old_idx}: "
                    f"parquet={src_parquet.exists()}, video={src_video.exists()}"
                )

            table = pq.read_table(src_parquet)
            n = table.num_rows
            if n != ep["length"]:
                raise ValueError(
                    f"{src_name} ep {old_idx}: parquet rows={n} != episodes.jsonl length={ep['length']}"
                )

            new_episode_col = pa.array([new_ep_index] * n, type=pa.int64())
            new_index_col = pa.array(
                range(global_frame_offset, global_frame_offset + n), type=pa.int64()
            )
            table = table.set_column(
                table.schema.get_field_index("episode_index"),
                "episode_index",
                new_episode_col,
            )
            table = table.set_column(
                table.schema.get_field_index("index"),
                "index",
                new_index_col,
            )

            dst_parquet = (
                DEST / "data" / "chunk-000" / f"episode_{new_ep_index:06d}.parquet"
            )
            pq.write_table(table, dst_parquet)

            dst_video = (
                DEST
                / "videos"
                / "chunk-000"
                / VIDEO_KEY
                / f"episode_{new_ep_index:06d}.mp4"
            )
            shutil.copy2(src_video, dst_video)

            merged_episodes.append(
                {
                    "episode_index": new_ep_index,
                    "tasks": ep["tasks"],
                    "length": n,
                }
            )

            stat = src_stats.get(old_idx)
            if stat is not None:
                stat = dict(stat)
                stat["episode_index"] = new_ep_index
                merged_stats.append(stat)

            print(
                f"  {src_name} ep {old_idx:>3} -> merged ep {new_ep_index:>3} "
                f"({n} frames, index {global_frame_offset}..{global_frame_offset + n - 1})"
            )

            new_ep_index += 1
            global_frame_offset += n
            total_frames += n

    with open(DEST / "meta" / "episodes.jsonl", "w") as f:
        for ep in merged_episodes:
            f.write(json.dumps(ep) + "\n")

    with open(DEST / "meta" / "episodes_stats.jsonl", "w") as f:
        for stat in merged_stats:
            f.write(json.dumps(stat) + "\n")

    shutil.copy2(
        BASE / SOURCES[0] / "meta" / "tasks.jsonl", DEST / "meta" / "tasks.jsonl"
    )
    shutil.copy2(
        BASE / SOURCES[0] / "meta" / "modality.json", DEST / "meta" / "modality.json"
    )

    with open(BASE / SOURCES[0] / "meta" / "info.json") as f:
        info = json.load(f)
    info["total_episodes"] = new_ep_index
    info["total_frames"] = total_frames
    info["total_videos"] = new_ep_index
    info["total_chunks"] = 1
    info["splits"] = {"train": f"0:{new_ep_index}"}
    info["discarded_episode_indices"] = []
    info["merged_from"] = SOURCES
    with open(DEST / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=4)

    print()
    print(f"Merged {new_ep_index} episodes / {total_frames} frames -> {DEST}")


if __name__ == "__main__":
    main()
