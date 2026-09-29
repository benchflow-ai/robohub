"""Trusted episode server (compatibility module).

The episode server now lives in BenchFlow (`benchflow.embodied.server`): budgets, recording, the per-step trace,
judging, the oracle token, vision mode and roles. robouse builds it for a task with `robouse.embodied`.
`serve(...)` keeps the signature `robouse serve` and the local runner use.

Episode files written to the run directory: result.json, episode.json, steps.jsonl, trace.jsonl, frames.jsonl,
video_index.json, recording.mp4 (see BenchFlow's docs/embodied.md).
"""
from __future__ import annotations

from pathlib import Path

from benchflow.embodied.server import EpisodeServer
from benchflow.embodied.server import serve as _serve

from .embodied import episode_from_spec

Episode = EpisodeServer


def serve(spec: dict, run_dir: Path, sock_path: str, workspace: Path | None = None, max_wall_s: float = 1800,
          ready_file: Path | None = None, seed: int | None = None) -> dict:
    ep = episode_from_spec(spec, Path(run_dir), workspace, seed=seed, max_wall_s=max_wall_s)
    return _serve(ep, sock_path, ready_file=ready_file)
