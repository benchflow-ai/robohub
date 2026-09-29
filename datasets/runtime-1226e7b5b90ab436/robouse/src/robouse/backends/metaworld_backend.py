"""Meta-World (Farama, v3) backend: 50 Sawyer manipulation tasks in MuJoCo.

Action: [dx, dy, dz, grip] in [-1, 1]. dx/dy/dz move the end effector (about 1 cm per unit per step);
grip > 0 closes the gripper, grip < 0 opens it.

Observation (fully observable, as in Meta-World's MT benchmarks with the goal shown): hand position,
gripper opening, the first object's position and quaternion, the second object's pose, and the goal.
The raw 39-float vector is also returned as `obs_vector`, which is what Meta-World's scripted experts read.
"""
from __future__ import annotations

import numpy as np

from .base import ActionSpec, Backend, StepInfo, G, arm_layout



def _env_class(env_name: str):
    from metaworld.env_dict import ALL_V3_ENVIRONMENTS

    if env_name not in ALL_V3_ENVIRONMENTS:
        raise KeyError(f"unknown Meta-World env {env_name!r}")
    return ALL_V3_ENVIRONMENTS[env_name]


class MetaWorldBackend(Backend):
    name = "metaworld"
    robot_name = "sawyer-2f"
    dense_reward = "shaped"  # Meta-World's own shaped reward

    def action_layout(self) -> list[G]:
        return arm_layout("end-effector velocity, about 1 cm per unit per step")

    def __init__(self, env_name: str, camera: str = "corner", max_steps: int = 500, width: int = 320, height: int = 320):
        import metaworld

        self.env_name = env_name
        self.camera = camera
        self.max_steps = max_steps
        self._w, self._h = width, height
        self._mt1 = metaworld.MT1(env_name, seed=0)
        self.env = _env_class(env_name)()
        self._renderer = None
        self.action_spec = ActionSpec(
            names=["dx", "dy", "dz", "grip"], low=[-1, -1, -1, -1], high=[1, 1, 1, 1],
            doc="dx/dy/dz: end-effector velocity, about 1 cm per unit per step; grip: +1 close, -1 open",
        )
        self._obs = None
        self._info: dict = {}
        self._grip = -1.0

    def reset(self, seed: int) -> None:
        tasks = [t for t in self._mt1.train_tasks if t.env_name == self.env_name]
        self.env.set_task(tasks[seed % len(tasks)])
        self.env._partially_observable = False  # show the goal, as in the fully-observable MT setting
        # the episode server enforces the step budget; lift Meta-World's own 500-step truncation, which
        # otherwise raises on the next step ("You must reset the env manually once truncate==True")
        self.env.max_path_length = 10**9
        self._obs, _ = self.env.reset(seed=seed)
        for _ in range(3):  # let objects settle so the first observation matches the scene (not counted as steps)
            self._obs, *_ = self.env.step(np.array([0.0, 0.0, 0.0, -1.0], dtype=np.float32))
        self._info = {"success": 0.0}
        self._grip = -1.0

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        self._grip = float(a[3])
        self._obs, r, _term, _trunc, info = self.env.step(a.astype(np.float32))
        self._info = info
        return StepInfo(success=bool(info.get("success", 0.0) >= 1.0), reward=float(r),
                        extra={k: float(v) for k, v in info.items() if isinstance(v, (int, float, np.floating))})

    def observe(self) -> dict:
        o = np.asarray(self._obs, dtype=float)
        r = lambda v: [round(float(x), 4) for x in v]
        return {
            "hand_pos": r(o[0:3]),
            "gripper_open": round(float(o[3]), 4),
            "obj1_pos": r(o[4:7]),
            "obj1_quat": r(o[7:11]),
            "obj2_pos": r(o[11:14]),
            "goal_pos": r(o[36:39]),
            "obs_vector": r(o),
        }

    def render(self, width: int = 320, height: int = 320) -> np.ndarray:
        """Offscreen render with MuJoCo's own renderer (works on macOS without gymnasium's GL setup)."""
        import mujoco

        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.env.model, self._h, self._w)
        self._renderer.update_scene(self.env.data, camera=self.camera)
        return self._renderer.render()[::-1].copy()  # Meta-World cameras come out upside down

    image_flipped = True  # render() flips vertically; camera projections account for it

    def mj_model_data(self):
        return self.env.model, self.env.data

    def success(self) -> bool:
        return bool(self._info.get("success", 0.0) >= 1.0)

    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self._obs[0:3], dtype=float)

    def expert_action(self) -> np.ndarray:
        """Scripted Meta-World expert, used only by the oracle solution."""
        from metaworld.policies import ENV_POLICY_MAP

        pol = ENV_POLICY_MAP[self.env_name]()
        return np.asarray(pol.get_action(self._obs), dtype=float)

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass
