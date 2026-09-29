"""Helpers shared by the GPU-track simulator modules (run on the GPU worker only)."""
from __future__ import annotations

import base64
import io
import math

import numpy as np

JPEG_QUALITY = 90


def jpeg(img) -> str:
    from PIL import Image

    a = np.asarray(img)
    if a.dtype != np.uint8:
        a = np.clip(a * (255 if a.max() <= 1.0 else 1), 0, 255).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(a[..., :3]).save(buf, format="JPEG", quality=JPEG_QUALITY)
    return base64.b64encode(buf.getvalue()).decode()


def r3(v, n: int = 4) -> list[float]:
    return [round(float(x), n) for x in np.asarray(v, dtype=float).ravel()]


def quat_to_rpy_deg(q_wxyz) -> list[float]:
    """Extrinsic x-y-z (roll, pitch, yaw) in degrees from a unit quaternion [w, x, y, z]."""
    w, x, y, z = [float(c) for c in q_wxyz]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [round(math.degrees(a), 2) for a in (roll, pitch, yaw)]


def rpy_deg_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    """Inverse of quat_to_rpy_deg: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), returned as [w, x, y, z]."""
    r, p, y = (math.radians(a) / 2 for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def pose_dict(p, q_wxyz) -> dict:
    """A pose as the agent sees it: position (m), quaternion [w, x, y, z] and roll/pitch/yaw in degrees."""
    return {"pos": r3(p), "quat_wxyz": r3(q_wxyz), "rpy_deg": quat_to_rpy_deg(q_wxyz)}


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def projection(K, world_to_cam_cv) -> list[list[float]]:
    """3x4 P with [u*w, v*w, w] = P @ [x, y, z, 1] in image pixels (OpenCV camera convention)."""
    P = np.asarray(K, dtype=float) @ np.asarray(world_to_cam_cv, dtype=float)[:3, :4]
    return [[round(float(v), 5) for v in row] for row in P]


def parse_floats(args, n_min: int, n_max: int, what: str) -> list[float]:
    try:
        v = [float(a) for a in args]
    except ValueError:
        raise ValueError(f"{what}: arguments must be numbers, got {args}")
    if not n_min <= len(v) <= n_max:
        raise ValueError(f"{what}: expected {n_min}-{n_max} numbers, got {len(v)}")
    return v
