"""
Web-based episode browser for LeRobot-format datasets.

Browse a collected dataset in the browser: paginated grid of episodes (30 per
page), camera tab selector, per-card play/pause/restart, and a delete button
that marks the episode as discarded (appended to
``meta/info.json:discarded_episode_indices`` — the same flag the ``x`` key sets
during collection). No files are touched.

Usage:
    python gear_sonic/scripts/run_episode_browser.py \\
        --dataset outputs/2026-05-20-10-32-17 \\
        --port 8765

Then open http://localhost:8765 in a browser.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import re
import threading
import urllib.parse
from datetime import date
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
INDEX_HTML = HERE / "episode_browser_index.html"

DISCARD_LOCK = threading.Lock()


def _info_path(root: Path) -> Path:
    return root / "meta" / "info.json"


def _load_info(root: Path) -> dict[str, Any]:
    return json.loads(_info_path(root).read_text())


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


def _camera_keys(info: dict[str, Any]) -> list[str]:
    return [k for k, v in info.get("features", {}).items() if v.get("dtype") == "video"]


def _episode_file_paths(root: Path, info: dict[str, Any], idx: int) -> dict[str, Path]:
    chunks_size = info["chunks_size"]
    chunk = idx // chunks_size
    paths = {"parquet": root / "data" / f"chunk-{chunk:03d}" / f"episode_{idx:06d}.parquet"}
    for cam in _camera_keys(info):
        paths[cam] = root / "videos" / f"chunk-{chunk:03d}" / cam / f"episode_{idx:06d}.mp4"
    return paths


def collect_episodes(root: Path) -> dict[str, Any]:
    info = _load_info(root)
    eps = _load_jsonl(root / "meta" / "episodes.jsonl")
    discarded = set(info.get("discarded_episode_indices") or [])
    cameras = _camera_keys(info)
    rows = []
    for e in eps:
        idx = e["episode_index"]
        files = _episode_file_paths(root, info, idx)
        avail_cams = [c for c in cameras if files[c].exists()]
        rows.append(
            {
                "episode_index": idx,
                "length": e.get("length"),
                "tasks": e.get("tasks", []),
                "discarded": idx in discarded,
                "available_cameras": avail_cams,
            }
        )
    rows.sort(key=lambda r: r["episode_index"])
    return {
        "dataset": str(root),
        "total_episodes": info.get("total_episodes"),
        "total_frames": info.get("total_frames"),
        "fps": info.get("fps"),
        "cameras": cameras,
        "episodes": rows,
    }


def set_discarded(root: Path, target: int, discarded: bool) -> dict[str, Any]:
    """Add/remove `target` in info.json's `discarded_episode_indices`. Files untouched."""
    with DISCARD_LOCK:
        info = _load_info(root)
        eps = _load_jsonl(root / "meta" / "episodes.jsonl")
        if target not in {e["episode_index"] for e in eps}:
            raise KeyError(f"episode {target} not found in episodes.jsonl")
        current = list(info.get("discarded_episode_indices") or [])
        was_discarded = target in current
        if discarded and not was_discarded:
            current.append(target)
        elif not discarded and was_discarded:
            current = [d for d in current if d != target]
        info["discarded_episode_indices"] = sorted(set(current))
        _info_path(root).write_text(json.dumps(info, indent=4))
        return {
            "episode_index": target,
            "discarded": discarded,
            "changed": discarded != was_discarded,
            "total_discarded": len(info["discarded_episode_indices"]),
        }


class Handler(BaseHTTPRequestHandler):
    dataset_root: Path = Path()

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        # Quieter default logging.
        msg = format % args
        if "/video/" in msg and " 206 " in msg:
            return
        super().log_message(format, *args)

    # ---- routing helpers ----
    def _send_json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json({"error": message}, status=status)

    # ---- handlers ----
    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path in ("/", "/index.html"):
            self._serve_index()
            return
        if path == "/api/episodes":
            try:
                self._send_json(collect_episodes(self.dataset_root))
            except Exception as exc:  # noqa: BLE001
                self._send_error_json(500, f"failed to list episodes: {exc}")
            return
        m = re.match(r"^/video/([^/]+)/(\d+)\.mp4$", path)
        if m:
            cam = urllib.parse.unquote(m.group(1))
            idx = int(m.group(2))
            self._serve_video(cam, idx)
            return
        self._send_error_json(404, f"not found: {path}")

    def do_POST(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        m = re.match(r"^/api/episodes/(\d+)/discard$", parsed.path)
        if not m:
            self._send_error_json(404, f"not found: {parsed.path}")
            return
        idx = int(m.group(1))
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(length) if length > 0 else b""
            payload = json.loads(body) if body else {}
            discarded = bool(payload.get("discarded", True))
            result = set_discarded(self.dataset_root, idx, discarded)
            self._send_json(result)
        except KeyError as exc:
            self._send_error_json(404, str(exc))
        except Exception as exc:  # noqa: BLE001
            self._send_error_json(500, f"discard failed: {exc}")

    # ---- serve helpers ----
    def _serve_index(self) -> None:
        try:
            body = INDEX_HTML.read_bytes()
        except FileNotFoundError:
            self._send_error_json(500, f"index template missing: {INDEX_HTML}")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_video(self, cam: str, idx: int) -> None:
        try:
            info = _load_info(self.dataset_root)
        except FileNotFoundError:
            self._send_error_json(500, "dataset info.json missing")
            return
        if cam not in _camera_keys(info):
            self._send_error_json(404, f"unknown camera: {cam}")
            return
        path = _episode_file_paths(self.dataset_root, info, idx).get(cam)
        if path is None or not path.exists():
            self._send_error_json(404, f"video not found: {cam} episode {idx}")
            return
        self._stream_file_with_range(path)

    def _stream_file_with_range(self, path: Path) -> None:
        file_size = path.stat().st_size
        ctype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        range_header = self.headers.get("Range")
        if range_header:
            m = re.match(r"bytes=(\d*)-(\d*)", range_header)
            if not m:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{file_size}")
                self.end_headers()
                return
            start_s, end_s = m.group(1), m.group(2)
            if start_s == "":
                # suffix range
                suffix = int(end_s)
                start = max(0, file_size - suffix)
                end = file_size - 1
            else:
                start = int(start_s)
                end = int(end_s) if end_s else file_size - 1
            if start >= file_size or end >= file_size or start > end:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{file_size}")
                self.end_headers()
                return
            length = end - start + 1
            self.send_response(HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-Type", ctype)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
            self.send_header("Content-Length", str(length))
            self.end_headers()
            with path.open("rb") as f:
                f.seek(start)
                remaining = length
                chunk = 64 * 1024
                while remaining > 0:
                    data = f.read(min(chunk, remaining))
                    if not data:
                        break
                    self.wfile.write(data)
                    remaining -= len(data)
        else:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(file_size))
            self.end_headers()
            with path.open("rb") as f:
                while True:
                    data = f.read(64 * 1024)
                    if not data:
                        break
                    self.wfile.write(data)


def _default_dataset() -> Path | None:
    outputs = Path(__file__).resolve().parents[2] / "outputs"
    if not outputs.exists():
        return None
    today = date.today().strftime("%Y-%m-%d")
    candidates = sorted(
        (p for p in outputs.iterdir() if p.is_dir() and p.name.startswith(today)),
        reverse=True,
    )
    if candidates:
        return candidates[0]
    # fallback: most recently modified output dir
    all_dirs = [p for p in outputs.iterdir() if p.is_dir() and (p / "meta" / "info.json").exists()]
    if not all_dirs:
        return None
    return max(all_dirs, key=lambda p: p.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=None, help="Path to a LeRobot dataset root")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    root = args.dataset or _default_dataset()
    if root is None:
        raise SystemExit("Could not find any dataset. Pass --dataset PATH.")
    root = root.resolve()
    if not (root / "meta" / "info.json").exists():
        raise SystemExit(f"Not a LeRobot dataset (missing meta/info.json): {root}")

    Handler.dataset_root = root
    print(f"[episode-browser] serving dataset: {root}")
    print(f"[episode-browser] open http://{args.host}:{args.port}/ in a browser")

    class _Server(ThreadingHTTPServer):
        daemon_threads = True

    with _Server((args.host, args.port), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[episode-browser] shutting down")


if __name__ == "__main__":
    main()
