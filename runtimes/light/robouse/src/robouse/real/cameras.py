"""Camera sources for real rigs: an OpenCV device index or path, or an MJPEG HTTP stream (e.g. ~/benchflow/robot's
cam_server.py on another machine). Each camera runs in its own thread and keeps only the newest frame, so a slow camera
never stalls the control loop; `frame()` returns that frame (RGB, resized to the configured size).

Spec strings (task `real.cameras`: {name: spec}): "0" / "1" (OpenCV index), "/dev/video2", "http://host:8000/cam/0.mjpg".
"""
from __future__ import annotations

import threading
import time
import urllib.request

import numpy as np


class Camera(threading.Thread):
    def __init__(self, name: str, spec: str, width: int = 640, height: int = 480):
        super().__init__(daemon=True)
        self.name, self.spec, self.size = name, str(spec), (int(width), int(height))
        self._frame: np.ndarray | None = None
        self._t = 0.0
        self._stop = threading.Event()
        self.error = ""

    def run(self) -> None:  # pragma: no cover - needs a camera
        try:
            if self.spec.startswith(("http://", "https://")):
                self._run_mjpeg()
            else:
                self._run_cv()
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"

    def _store(self, bgr_or_rgb: np.ndarray, is_bgr: bool) -> None:
        import cv2

        img = cv2.resize(bgr_or_rgb, self.size)
        self._frame = img[:, :, ::-1].copy() if is_bgr else img
        self._t = time.time()

    def _run_cv(self) -> None:  # pragma: no cover - needs a camera
        import cv2

        src = int(self.spec) if self.spec.lstrip("-").isdigit() else self.spec
        cap = cv2.VideoCapture(src)
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if ok:
                    self._store(frame, True)
                else:
                    time.sleep(0.05)
        finally:
            cap.release()

    def _run_mjpeg(self) -> None:  # pragma: no cover - needs a camera server
        import cv2

        with urllib.request.urlopen(self.spec, timeout=10) as r:
            buf = b""
            while not self._stop.is_set():
                buf += r.read(65536)
                a, b = buf.find(b"\xff\xd8"), buf.find(b"\xff\xd9")
                if a != -1 and b != -1 and b > a:
                    jpg, buf = buf[a:b + 2], buf[b + 2:]
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        self._store(img, True)

    def frame(self, max_age_s: float = 2.0) -> np.ndarray | None:
        if self._frame is None or time.time() - self._t > max_age_s:
            return None
        return self._frame

    def stop(self) -> None:
        self._stop.set()


def open_cameras(specs: dict, width: int = 640, height: int = 480) -> dict[str, Camera]:
    cams = {}
    for name, spec in (specs or {}).items():
        c = Camera(name, spec, width, height)
        c.start()
        cams[name] = c
    return cams
