"""Hidden verifier: presets=rgb / presets=depth pick the camera data type and the matching observation shape; default stays RGB."""

import pytest
from ss_isaac import resolve

TASK = "IsaacContrib-PoleCam-Direct"


def variant(overrides):
    e, _ = resolve(TASK, overrides)
    cam = e.scene.tiled_camera
    return list(cam.data_types), list(e.observation_space), (cam.width, cam.height)


def test_default_is_rgb():
    assert variant([]) == (["rgb"], [3, 128, 128], (128, 128))


def test_rgb_preset():
    assert variant(["presets=rgb"]) == (["rgb"], [3, 128, 128], (128, 128))


def test_depth_preset():
    types, obs, size = variant(["presets=depth"])
    assert types in (["depth"], ["distance_to_image_plane"], ["distance_to_camera"]), types
    assert obs == [1, 128, 128] and size == (128, 128)


def test_listed_as_domain_presets():
    from isaaclab_tasks.utils.hydra import PresetTarget, enumerate_task_presets

    p = enumerate_task_presets(TASK)
    assert p is not None and {"rgb", "depth"} <= set(p[PresetTarget.DOMAIN]), p


def test_not_a_physics_selector():
    with pytest.raises(BaseException):
        resolve(TASK, ["physics=depth"])
