"""Camera calibration for MuJoCo scenes: the 3x4 projection matrix of a camera in the saved image's pixels."""

from __future__ import annotations

from typing import Any

import numpy as np


def mujoco_camera(
    model: Any, data: Any, name: str, width: int, height: int, image_flipped: bool = False, calibrated: bool = True
) -> dict | None:
    """{name, width, height, fovy_deg, position, projection} with [u*w, v*w, w] = projection @ [x, y, z, 1]; None when
    the model has no camera `name`. An uncalibrated camera reports its size and field of view only."""
    import mujoco

    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if cid < 0:
        return None
    fovy = float(model.cam_fovy[cid])
    if not calibrated:
        return {"name": name, "width": width, "height": height, "fovy_deg": round(fovy, 2), "projection": None}
    f = (height / 2) / np.tan(np.deg2rad(fovy) / 2)
    rot = data.cam_xmat[cid].reshape(3, 3)
    pos = data.cam_xpos[cid]
    ext = np.hstack([rot.T, (-rot.T @ pos)[:, None]])
    k = np.array([[f, 0, -width / 2], [0, -f, -height / 2], [0, 0, -1.0]])
    proj = k @ ext
    if image_flipped:
        proj[1] = (height - 1) * proj[2] - proj[1]
    return {
        "name": name,
        "width": width,
        "height": height,
        "fovy_deg": round(fovy, 2),
        "position": [round(float(x), 4) for x in pos],
        "projection": [[round(float(x), 5) for x in row] for row in proj],
    }
