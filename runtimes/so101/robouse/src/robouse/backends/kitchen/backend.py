"""Franka Kitchen backend (Gymnasium-Robotics `FrankaKitchen-v1`, MuJoCo): a 9-DoF Franka Panda in a kitchen with a
microwave, a kettle, oven knobs, a light switch, a sliding cabinet and a hinged cabinet (Relay Policy Learning / D4RL).

The upstream action is 9 joint velocities; this backend adds an end-effector interface on top of it. Each step the agent
gives [dx, dy, dz, grip] in [-1, 1]: dx/dy/dz move a commanded gripper position by up to 3 cm per unit along world x/y/z,
and the backend solves one damped-least-squares inverse-kinematics step (position error + holding the starting gripper
orientation, with a null-space pull toward the starting arm posture) to get the joint velocities it passes to the
upstream `env.step`. The commanded position never runs more than 6 cm ahead of the measured gripper, so pushing
against furniture stalls instead of winding up. grip +1 closes the fingers, -1 opens them. One step = 80 ms (upstream).

Success is the upstream per-subtask completion check: for every goal subtask of the task, the norm of (the subtask's
joint values - its goal values) < BONUS_THRESH (0.3), evaluated on the current physical state (success_mode final).
"""

from __future__ import annotations

import numpy as np

from ..base import ActionSpec, Backend, StepInfo

POS_STEP = 0.03  # metres per action unit per step
ROLL_STEP = 0.1  # radians per action unit per step
ROLL_LIM = 1.6  # the commanded roll stays within +-ROLL_LIM of the starting orientation
LEAD = 0.06  # max distance of the commanded position ahead of the measured gripper
SUBTASKS = ["microwave", "kettle", "bottom burner", "top burner", "light switch", "slide cabinet", "hinge cabinet"]

# task id -> goal subtasks (in the order an agent may do them; any order is accepted)
TASKS: dict[str, list[str]] = {
    "kitchen-microwave": ["microwave"],
    "kitchen-kettle": ["kettle"],
    "kitchen-bottom-burner": ["bottom burner"],
    "kitchen-top-burner": ["top burner"],
    "kitchen-light-switch": ["light switch"],
    "kitchen-slide-cabinet": ["slide cabinet"],
    "kitchen-hinge-cabinet": ["hinge cabinet"],
    # the Minari/D4RL kitchen-complete and kitchen-partial/mixed goal sets
    "kitchen-complete": ["microwave", "kettle", "light switch", "slide cabinet"],
    "kitchen-mixed": ["microwave", "kettle", "bottom burner", "light switch"],
    "kitchen-burners-cabinets": ["bottom burner", "top burner", "slide cabinet", "hinge cabinet"],
}

# public landmarks: (observation key, MuJoCo site)
LANDMARKS = {
    "microwave_handle_pos": "microhandle_site",
    "kettle_handle_pos": "kettle_site",
    "bottom_burner_knob_pos": "knob2_site",
    "top_burner_knob_pos": "knob4_site",
    "light_switch_pos": "light_site",
    "slide_handle_pos": "slide_site",
    "hinge_handle_pos": "hinge_site2",
}

_R = lambda v: [round(float(x), 4) for x in np.ravel(v)]


def _key(sub: str) -> str:
    return sub.replace(" ", "_")


class KitchenBackend(Backend):
    name = "kitchen"

    def __init__(self, spec: dict, width: int = 320, height: int = 320):
        import gymnasium as gym
        import gymnasium_robotics
        import mujoco
        from gymnasium_robotics.envs.franka_kitchen import kitchen_env as ke

        self._ke = ke
        tasks = spec.get("tasks") or TASKS[str(spec.get("env") or spec["id"])]
        self.goals = [str(t) for t in tasks]
        self.max_steps = int(spec.get("max_steps", 400))
        self.camera = str(spec.get("camera", "front"))
        self._w, self._h = width, height
        gym.register_envs(gymnasium_robotics)
        # the episode server enforces the step budget and ends the episode; keep the upstream env running
        self.env = gym.make(
            "FrankaKitchen-v1",
            tasks_to_complete=self.goals,
            terminate_on_tasks_completed=False,
            remove_task_when_completed=False,
            max_episode_steps=10**9,
        )
        self.u = self.env.unwrapped
        self.m, self.d = self.u.model, self.u.data
        self._site = {k: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, s) for k, s in LANDMARKS.items()}
        self._ee = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        self._mw_hinge = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "microdoorroot")
        self._cab_hinge = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "hingerightdoor")
        self.action_spec = ActionSpec(
            names=["dx", "dy", "dz", "droll", "grip"],
            low=[-1] * 5,
            high=[1] * 5,
            doc="dx/dy/dz: move the gripper up to 3 cm per unit per step along world x/y/z; droll: turn the gripper "
            "about its pointing axis (+y) by up to 0.1 rad per unit per step (+ = its top toward +x); "
            "grip: +1 close, -1 open. One step = 80 ms",
        )
        self._renderer = None
        self._grip = -1.0

    # ---- core API -------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        self.env.reset(seed=int(seed))
        self._grip = -1.0
        self._q0 = self.d.qpos[:7].copy()
        self._R0 = self.d.site_xmat[self._ee].reshape(3, 3).copy()
        self._target = self.hand_pos()
        self._roll = 0.0

    def hand_pos(self) -> np.ndarray:
        return self.d.site_xpos[self._ee].copy()

    def _ik_velocity(self) -> np.ndarray:
        """Joint velocities (rad/s) for one damped-least-squares IK step toward the commanded pose."""
        import mujoco

        m, d = self.m, self.d
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jp, jr, self._ee)
        J = np.vstack([jp[:, :7], jr[:, :7]])
        R = d.site_xmat[self._ee].reshape(3, 3)
        e_pos = self._target - d.site_xpos[self._ee]
        c, s = np.cos(self._roll), np.sin(self._roll)
        Ry = np.array(
            [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        )  # roll about world +y (the gripper's pointing axis at the start)
        Rt = Ry @ self._R0
        e_rot = 0.5 * sum(np.cross(R[:, i], Rt[:, i]) for i in range(3))
        e = np.concatenate([e_pos, 0.5 * e_rot])
        q = d.qpos[:7]
        lo, hi = m.jnt_range[:7, 0] + 0.05, m.jnt_range[:7, 1] - 0.05
        free = np.ones(7, dtype=bool)
        for _ in range(3):  # joints at a limit that the step would push further are frozen and the step re-solved
            Jf = J * free
            Jp = Jf.T @ np.linalg.inv(Jf @ Jf.T + 0.05**2 * np.eye(6))
            dq = Jp @ e + (np.eye(7) - Jp @ Jf) @ (0.1 * (self._q0 - q) * free)
            bad = free & (((q <= lo) & (dq < 0)) | ((q >= hi) & (dq > 0)))
            if not bad.any():
                break
            free &= ~bad
        dq = dq * free
        return dq / self.u.robot_env.dt

    def step(self, action) -> StepInfo:
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        if a.shape[0] == 4:  # the shared move_to/grip skills send [dx, dy, dz, grip]: keep the roll
            a = np.r_[a[:3], 0.0, a[3]]
        self._grip = float(a[4])
        self._roll = float(np.clip(self._roll + a[3] * ROLL_STEP, -ROLL_LIM, ROLL_LIM))
        hand = self.hand_pos()
        tgt = self._target + a[:3] * POS_STEP
        off = tgt - hand
        n = np.linalg.norm(off)
        if n > LEAD:
            tgt = hand + off * (LEAD / n)
        self._target = tgt
        qd = self._ik_velocity()
        width = 0.04 * (1.0 - self._grip) / 2.0  # per-finger opening target (0 closed .. 0.04 open)
        fd = (width - self.d.qpos[7:9]) / self.u.robot_env.dt
        vel = np.concatenate([qd, fd]) / 2.0  # upstream action = velocity / 2 rad/s, clipped to [-1, 1]
        _obs, r, _t, _tr, _info = self.env.step(np.clip(vel, -1, 1))
        return StepInfo(success=self.success(), reward=float(r))

    def roll(self) -> float:
        """Measured roll of the gripper about world +y relative to the starting orientation (rad)."""
        R = self.d.site_xmat[self._ee].reshape(3, 3) @ self._R0.T
        return float(np.arctan2(R[0, 2], R[2, 2]))

    def subtask_state(self, sub: str) -> tuple[np.ndarray, np.ndarray, float]:
        idx = self._ke.OBS_ELEMENT_INDICES[sub]
        cur = self.d.qpos[idx].copy()
        goal = self._ke.OBS_ELEMENT_GOALS[sub]
        return cur, goal, float(np.linalg.norm(cur - goal))

    def success(self) -> bool:
        return all(self.subtask_state(s)[2] < self._ke.BONUS_THRESH for s in self.goals)

    def observe(self) -> dict:
        d = self.d
        out = {
            "hand_pos": _R(self.hand_pos()),
            "hand_target": _R(self._target),
            "hand_roll": round(self.roll(), 4),
            "hand_roll_target": round(self._roll, 4),
            "gripper_open": round(float(np.clip(d.qpos[7:9].sum() / 0.08, 0, 1)), 4),
            "gripper_width": round(float(d.qpos[7:9].sum()), 4),
            "arm_joints": _R(d.qpos[:7]),
        }
        for k, sid in self._site.items():
            out[k] = _R(d.site_xpos[sid])
        out["microwave_hinge_pos"] = _R(d.xpos[self._mw_hinge])
        out["hinge_cabinet_hinge_pos"] = _R(d.xpos[self._cab_hinge])
        out["kettle_pos"] = _R(d.qpos[23:26])
        out["kettle_quat"] = _R(d.qpos[26:30])
        out["kettle_goal_pos"] = _R(self._ke.OBS_ELEMENT_GOALS["kettle"][:3])
        subs = {}
        for s in SUBTASKS:
            cur, goal, dist = self.subtask_state(s)
            subs[_key(s)] = {
                "joints": _R(cur),
                "goal": _R(goal),
                "distance": round(dist, 4),
                "complete": bool(dist < self._ke.BONUS_THRESH),
                "required": s in self.goals,
            }
        out["subtasks"] = subs
        out["goals_complete"] = (
            f"{sum(self.subtask_state(s)[2] < self._ke.BONUS_THRESH for s in self.goals)}/{len(self.goals)}"
        )
        return out

    # ---- rendering ------------------------------------------------------------------------------
    def mj_model_data(self):
        return self.m, self.d

    def render(self, width: int = 320, height: int = 320) -> np.ndarray:
        import mujoco

        if self._renderer is None:
            # Software rendering (OSMesa, in the hub's simulator image) spends most of a frame on shadow maps and
            # multisampling (about 3.4 s instead of 0.2-0.5 s per kitchen frame), so both are off; physics is unaffected.
            self.m.vis.quality.offsamples = 0
            self._renderer = mujoco.Renderer(self.m, self._h, self._w)
            self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            # without shadows the oven hood's spotlight would flood the stove top: switch that light off (visual only)
            self.m.light_active[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_LIGHT, "ovenlight")] = 0
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.distance, cam.azimuth, cam.elevation = 2.2, 70.0, -35.0  # the upstream default view
            cam.lookat[:] = [-0.2, 0.5, 2.0]
            self._cam = cam
        mujoco.mj_forward(self.m, self.d)
        self._renderer.update_scene(self.d, camera=self._cam)
        return self._renderer.render().copy()

    def record_camera(self):
        if self._renderer is None:  # sets up the free camera (and the visual-only light tweak)
            self.render()
        return self._cam

    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def close(self) -> None:
        try:
            if self._renderer is not None:
                self._renderer.close()
            self.env.close()
        except Exception:  # noqa: BLE001 - best-effort teardown
            pass


# ---- reference solution: scripted end-effector waypoints per subtask (socket client only; public observation fields) ----


class _Finished(Exception):
    pass


def _call(req: dict) -> dict:
    from robouse.agent_cli import _send

    r = _send(req)
    if not r.get("ok"):
        raise RuntimeError(r.get("error"))
    res = r["result"]
    if "episode" in res:
        raise _Finished(res["episode"])
    return res


def _state() -> dict:
    return _call({"op": "observe"})["state"]


def _act(dx, dy, dz, droll, grip, repeat: int = 1) -> dict:
    return _call(
        {"op": "act", "action": [float(dx), float(dy), float(dz), float(droll), float(grip)], "repeat": int(repeat)}
    )["state"]


def _move(pos, grip: float, tol: float = 0.01, max_steps: int = 80) -> dict:
    return _call(
        {"op": "move_to", "pos": [float(x) for x in pos], "grip": float(grip), "tol": tol, "max_steps": max_steps}
    )["state"]


def _grip(v: float, steps: int = 8) -> dict:
    return _call({"op": "grip", "value": float(v), "steps": int(steps)})["state"]


def _roll_to(target: float, grip: float) -> dict:
    st = _state()
    for _ in range(40):
        err = target - st["hand_roll_target"]
        if abs(err) < 0.02:
            break
        st = _act(0, 0, 0, np.clip(err / ROLL_STEP, -1, 1), grip)
    return _act(0, 0, 0, 0, grip, 4)


def _sub(st: dict, key: str) -> dict:
    return st["subtasks"][key]


def _retreat(st: dict, grip: float = -1.0) -> dict:
    """Back straight away from the furniture (-y), then up to a free height in front of the kitchen."""
    h = np.array(st["hand_pos"])
    st = _move(h + [0, -0.1, 0], grip, tol=0.02, max_steps=30)
    h = np.array(st["hand_pos"])
    return _move([h[0], min(h[1], 0.35), 2.1], grip, tol=0.03, max_steps=60)


def _arc(center, handle, key: str, sign: float, done, step: float = 0.05, grip: float = 1.0) -> dict:
    """Carry a grasped door handle around its vertical hinge axis until done(state) holds."""
    c = np.asarray(center[:2], dtype=float)
    h = np.asarray(handle, dtype=float)
    r0 = h[:2] - c
    rad, a = float(np.linalg.norm(r0)), float(np.arctan2(r0[1], r0[0]))
    st = _state()
    for _ in range(45):
        if (
            done(st) or np.linalg.norm(np.array(st[key]) - np.array(st["hand_pos"])) > 0.05
        ):  # done, or the handle slipped
            break
        a += sign * step
        st = _move(np.r_[c + rad * np.array([np.cos(a), np.sin(a)]), h[2]], grip, tol=0.01, max_steps=8)
    return st


def _microwave(st: dict) -> dict:
    hp = np.array(st["microwave_handle_pos"])
    _move(hp + [0, -0.12, 0], -1)
    _move(hp, -1, tol=0.008)
    _grip(1, 8)
    st = _arc(
        st["microwave_hinge_pos"], hp, "microwave_handle_pos", -1.0, lambda s: _sub(s, "microwave")["joints"][0] < -0.65
    )
    _grip(-1, 5)
    return _retreat(_state())


def _hinge_cabinet(st: dict) -> dict:
    hp = np.array(st["hinge_handle_pos"])
    _move(hp + [0, -0.12, 0], -1)
    _move(hp, -1, tol=0.008)
    _grip(1, 8)
    st = _arc(
        st["hinge_cabinet_hinge_pos"],
        hp,
        "hinge_handle_pos",
        1.0,
        lambda s: _sub(s, "hinge_cabinet")["joints"][1] > 1.42,
    )
    _grip(-1, 5)
    return _retreat(_state())


def _slide_cabinet(st: dict) -> dict:
    sp = np.array(st["slide_handle_pos"])
    _move(sp + [0, -0.12, 0], -1)
    _move(sp, -1, tol=0.008)
    st = _grip(1, 8)
    for _ in range(20):
        if _sub(st, "slide_cabinet")["joints"][0] > 0.4:
            break
        st = _move(np.array(st["hand_pos"]) + [0.04, 0, 0], 1, tol=0.01, max_steps=8)
    _grip(-1, 5)
    return _retreat(_state())


def _knob(st: dict, pos_key: str, key: str, goal: float) -> dict:
    kp = np.array(st[pos_key])
    _roll_to(0.0, -1)
    _move(kp + [0, -0.12, 0], -1)
    _move(kp + [0, -0.035, 0], -1, tol=0.008)
    st = _grip(1, 8)
    for _ in range(60):  # turn the knob by turning the wrist (+roll) until the knob angle is just past the goal
        if _sub(st, key)["joints"][0] < goal + 0.03:
            break
        st = _act(0, 0, 0, 0.5, 1)
    _grip(-1, 5)
    st = _retreat(_state())
    return _roll_to(0.0, -1)


def _light_switch(st: dict) -> dict:
    lp = np.array(st["light_switch_pos"])
    _grip(1, 5)
    _move(lp + [0.05, -0.12, 0], 1)
    st = _move(lp + [0.05, -0.05, 0], 1, tol=0.008)
    for k in range(20):  # sweep the lever tip toward -x
        if _sub(st, "light_switch")["joints"][0] < -0.66:
            break
        st = _move(
            np.array(st["hand_pos"]) * [0, 1, 1] + [lp[0] + 0.05 - 0.012 * (k + 1), 0, 0], 1, tol=0.005, max_steps=6
        )
    return _retreat(st, 1)


def _kettle(st: dict) -> dict:
    kh = np.array(st["kettle_handle_pos"])
    goal = np.array(st["kettle_goal_pos"])
    _move(kh + [0, -0.15, 0], -1)
    _roll_to(np.pi / 2, -1)  # fingers above and below the handle bar (which runs along x)
    _move(kh, -1, tol=0.008)
    _grip(1, 10)
    st = _move(kh + [0, 0, 0.08], 1)
    for _ in range(3):
        off = np.array(st["hand_pos"]) - np.array(st["kettle_pos"])
        st = _move(np.r_[goal[:2], goal[2] + 0.08] + off, 1, tol=0.01, max_steps=60)
        if np.linalg.norm(np.array(st["kettle_pos"])[:2] - goal[:2]) < 0.02:
            break
    off = np.array(st["hand_pos"]) - np.array(st["kettle_pos"])
    _move(goal + off + [0, 0, 0.01], 1, tol=0.01, max_steps=40)
    st = _grip(-1, 8)
    st = _retreat(st)
    return _roll_to(0.0, -1)


SOLVERS = {
    "microwave": _microwave,
    "kettle": _kettle,
    "bottom burner": lambda st: _knob(st, "bottom_burner_knob_pos", "bottom_burner", -0.88),
    "top burner": lambda st: _knob(st, "top_burner_knob_pos", "top_burner", -0.92),
    "light switch": _light_switch,
    "slide cabinet": _slide_cabinet,
    "hinge cabinet": _hinge_cabinet,
}


def oracle_main(env: str) -> None:
    from robouse.agent_cli import _send

    try:
        _state()
        for sub in TASKS[env]:
            SOLVERS[sub](_state())
        _act(0, 0, 0, 0, -1, 5)
    except _Finished:
        return
    _send({"op": "done", "text": "oracle finished"})
