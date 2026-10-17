"""The episode video: frames written as they are rendered, with the wall-clock time and env step of each."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from .. import config

FPS = 30


def record_size(value: str | None = None) -> tuple[int, int] | None:
    """(width, height) from 'WIDTHxHEIGHT' (default: $ROBOUSE_RECORD_SIZE); None when unset. Even sizes only."""
    v = (config.env("ROBOUSE_RECORD_SIZE") if value is None else value).strip().lower()
    if not v:
        return None
    try:
        w, h = (int(x) for x in v.split("x"))
    except ValueError:
        raise ValueError(f"record size must look like 1280x720, got {v!r}") from None
    if not (16 <= w <= 3840 and 16 <= h <= 2160):
        raise ValueError(f"record size out of range (16..3840 x 16..2160): {v!r}")
    return w - w % 2, h - h % 2


class VideoRecorder:
    """Writes recording.mp4 and frames.jsonl in `run_dir`. The video opens on the first frame; later frames of another
    size are resized to it. A chosen size (`exact=True`) is kept exactly and encoded near-lossless; the native path
    keeps the default encoding."""

    def __init__(self, run_dir: Path, t0: float, exact: bool = False):
        self.run_dir, self.t0, self.exact = Path(run_dir), t0, exact
        self._writer: Any = None
        self.size: tuple[int, int] | None = None
        self.frames: list[dict] = []

    def add(self, img: Any, step: int) -> None:
        img = np.asarray(img, dtype=np.uint8)
        if self._writer is None:
            import imageio.v2 as imageio

            self.size = (img.shape[1], img.shape[0])
            enc: dict[str, Any] = (
                {"quality": None, "macro_block_size": 2, "output_params": ["-crf", "18", "-preset", "medium"]}
                if self.exact
                else {"quality": 7, "macro_block_size": 16}
            )
            self._writer = imageio.get_writer(
                self.run_dir / "recording.mp4", fps=FPS, codec="libx264", ffmpeg_log_level="error", **enc
            )
        elif (img.shape[1], img.shape[0]) != self.size:
            from PIL import Image

            img = np.asarray(Image.fromarray(img).resize(self.size))  # type: ignore[arg-type]  # set with the writer
        self._writer.append_data(img)
        self.frames.append({"i": len(self.frames), "t": round(time.time() - self.t0, 3), "step": step})

    def close(self, keep: bool = True) -> None:
        """Finish the video; `keep=False` (an episode in which no simulator step ran) removes it, since a still frame
        is not a recording of anything."""
        if self._writer is not None:
            self._writer.close()
            self._writer = None
        if not keep:
            (self.run_dir / "recording.mp4").unlink(missing_ok=True)
            self.frames, self.size = [], None
        with open(self.run_dir / "frames.jsonl", "w") as f:
            for m in self.frames:
                f.write(json.dumps(m) + "\n")
