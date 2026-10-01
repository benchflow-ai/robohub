"""robouse on BenchFlow's embodied layer (`benchflow.embodied`).

BenchFlow owns the episode server, the `robo` protocol and command, the simulator-sidecar wiring, the physical
verifier and the rollout traces. robouse keeps the simulator backends; this module adapts them:

  RobouseSim             a robouse Backend as a `benchflow.embodied.SimBackend`
  episode_from_spec      the EpisodeServer for a task's `robouse:` block (used by `robouse serve`)
  episode_from_task      the same for a task folder; the simulator sidecar runs it through
                         `python -m benchflow.embodied.serve --factory robouse.engine.sim:episode_from_task`
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from benchflow.embodied.backend import StepResult
from benchflow.embodied.server import LEGACY_VISIBLE, EpisodeConfig, EpisodeServer
from benchflow.embodied.spec import Embodiment

from ..backends import make_backend
from ..backends.base import Backend, embodiment_of

_FORWARD = ("judge", "run_skill", "start_skill", "pop_frames", "workspace_images", "privileged_state")


class RobouseSim:
    """Adapts a robouse `Backend` to BenchFlow's `SimBackend` contract.

    Built-in skills drive the backend through its protocol-1 skill action, a 4-number [dx, dy, dz, grip] vector for
    the active arm (every robouse backend accepts it), so skill behaviour is exactly what robouse had before."""

    def __init__(self, backend: Backend):
        self.b = backend
        self.name = backend.name
        self.default_camera = getattr(backend, "camera", None) or None
        self.probe_success = bool(getattr(backend, "probe_success", True))
        self.open_skill_set = backend.name == "behavior"
        for attr in _FORWARD:
            fn = getattr(backend, attr, None)
            if callable(fn):
                setattr(self, attr, fn)
        self._embodiment = Embodiment.from_dict(embodiment_of(backend).to_dict())
        # embodiment backends (backends/embodied.py) say what "hold still" is for their action groups
        self._hold = getattr(backend, "hold_action", None)

    # ---- SimBackend -----------------------------------------------------------------------------------------
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def reset(self, seed: int) -> None:
        self.b.reset(seed)

    def step(self, action) -> StepResult:
        info = self.b.step(action)
        return StepResult(success=bool(info.success), reward=float(info.reward), info=dict(info.extra or {}))

    def observe(self) -> dict[str, Any]:
        return self.b.observe()

    def render(self, camera: str | None = None):
        if not camera or camera == getattr(self.b, "camera", None):
            return self.b.render()
        old = self.b.camera
        try:
            self.b.camera = camera
            return self.b.render()
        finally:
            self.b.camera = old

    def success(self) -> bool:
        return bool(self.b.success())

    # ---- optional capabilities --------------------------------------------------------------------------------
    def ee_position(self, arm: str | None = None):
        return [float(x) for x in self.b.hand_pos()]

    def ee_command(self, arm: str | None, delta, grip: float | None) -> list[float]:
        g = float(getattr(self.b, "_grip", -1.0) if grip is None else grip)
        return [float(x) for x in delta] + [g]

    @property
    def last_judge(self):
        return getattr(self.b, "last_judge", None)

    def hold_action(self) -> list[float]:
        if callable(self._hold):
            return [float(x) for x in self._hold()]
        g = float(getattr(self.b, "_grip", 0.0))
        dim = self.b.action_spec.dim
        return [0.0] * (dim - 1) + [g] if dim >= 1 else []

    def legacy_skill_names(self) -> list[str]:
        return list(self.b.skills())

    def camera_calibration(self, name: str, width: int, height: int) -> dict | None:
        m_d = getattr(self.b, "mj_model_data", lambda: None)()
        if not m_d:
            return None
        from benchflow.embodied.cameras import mujoco_camera

        model, data = m_d
        return mujoco_camera(
            model, data, name, width, height, image_flipped=bool(getattr(self.b, "image_flipped", False))
        )

    def close(self) -> None:
        self.b.close()


def episode_from_spec(
    spec: dict, run_dir: Path, workspace: Path | None = None, seed: int | None = None, max_wall_s: float | None = None
) -> EpisodeServer:
    sim = RobouseSim(make_backend(spec))
    block = dict(spec)
    block.setdefault("visible_fields", list(LEGACY_VISIBLE))  # protocol-1 default for vision tasks
    # robouse's `settle_steps` is a backend setting (LIBERO: still steps at reset), not the post-`done` settle
    block.pop("settle_steps", None)
    cfg = EpisodeConfig.from_task(
        block,
        sim.embodiment(),
        seed=seed,
        default_camera=sim.default_camera,
        max_wall_s=1800.0 if max_wall_s is None else max_wall_s,
    )
    return EpisodeServer(sim, cfg, run_dir, workspace)


def episode_from_task(
    task_dir: Path,
    *,
    run_dir: Path,
    workspace: Path | None = None,
    seed: int | None = None,
    max_wall_s: float | None = None,
) -> EpisodeServer:
    from ..tasks import load_task

    return episode_from_spec(load_task(task_dir).spec, run_dir, workspace, seed, max_wall_s)
