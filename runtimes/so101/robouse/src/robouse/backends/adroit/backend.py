"""Adroit hand backend (Gymnasium-Robotics `AdroitHand{Door,Hammer,Pen,Relocate}-v1`, MuJoCo; DAPG, Rajeswaran et al. 2018).

A 24-joint Shadow-style hand (the ADROIT model: wrist 2, first/middle/ring finger 4 each, little finger 5, thumb 5 joints,
one position actuator per joint) on an arm: Door 4 arm actuators (ARTz slide, ARRx/ARRy/ARRz), Hammer 2 (ARRx/ARRy), Pen none (the hand is fixed),
Relocate 6 (ARTx/ARTy/ARTz slides, ARRx/ARRy/ARRz). Every actuator is a position servo.

Action (one number per actuator, upstream order): the change of that actuator's position target this step, in the
actuator's own units (rad for hinges, m for slides), clipped so the target stays inside the actuator's control range.
All zeros holds the current targets. The upstream action (absolute targets normalised to [-1, 1]) is exactly
`(target - mid) / half_range`, so this is the upstream control with the target made incremental. One step = 10 ms
(upstream: 5 physics steps of 2 ms).

Tasks start from the initial state of one recorded DAPG human demonstration (Minari `D4RL/<env>/human-v2`), restored with
the env's own `set_env_state`; the reference solution replays that demonstration's actions.

Success is the env's own `info["success"]` after the step: Door hinge >= 1.35 rad; Hammer nail within 1 cm of the
fully-driven position; Pen orientation similarity > 0.95 with the pen within 7.5 cm of its start point; Relocate ball
within 10 cm of the target. success_mode final: judged after `robo done` and 10 steps holding the targets.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..base import ActionSpec, Backend, StepInfo

ENVS = {
    "door": "AdroitHandDoor-v1",
    "hammer": "AdroitHandHammer-v1",
    "pen": "AdroitHandPen-v1",
    "relocate": "AdroitHandRelocate-v1",
}
FINGERS = ["ff", "mf", "rf", "lf", "th"]

_R = lambda v, n=4: [round(float(x), n) for x in np.ravel(v)]


def _env_kind(env_id: str) -> str:
    for k, v in ENVS.items():
        if env_id in (k, v):
            return k
    raise KeyError(env_id)


class AdroitBackend(Backend):
    name = "adroit"

    def __init__(self, spec: dict, width: int = 320, height: int = 320):
        import warnings

        import gymnasium as gym
        import gymnasium_robotics
        import mujoco

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            gym.register_envs(gymnasium_robotics)
        self.kind = _env_kind(str(spec["env"]))
        self.env_id = ENVS[self.kind]
        self.init_state = spec.get("init_state")
        self.max_steps = int(spec.get("max_steps", 600))
        self.camera = str(spec.get("camera", "fixed"))
        self._w, self._h = width, height
        self.env = gym.make(self.env_id, max_episode_steps=10**9)
        self.u = self.env.unwrapped
        self.m, self.d = self.u.model, self.u.data
        m = self.m
        self.act_names = [m.actuator(i).name.removeprefix("A_") for i in range(m.nu)]
        # Targets are expressed as the joint position each servo settles at: upstream servos apply
        # gain * ctrl - k_bias * q, so q settles at ctrl * gain / k_bias (x2.5 for the arm, x1 for the hand).
        self.k = m.actuator_gainprm[:, 0] / -m.actuator_biasprm[:, 1]
        self.lo = m.actuator_ctrlrange[:, 0] * self.k
        self.hi = m.actuator_ctrlrange[:, 1] * self.k
        self.jnt_qadr = [int(m.jnt_qposadr[m.actuator_trnid[i, 0]]) for i in range(m.nu)]
        self.jnt_dadr = [int(m.jnt_dofadr[m.actuator_trnid[i, 0]]) for i in range(m.nu)]
        width_ = self.hi - self.lo
        self.action_spec = ActionSpec(
            names=self.act_names,
            low=_R(-width_),
            high=_R(width_),
            doc="one number per actuator: the change of its position target this step (rad for hinges, m for the arm "
            "slides ARTx/ARTy/ARTz); the target stays inside the actuator's range; all zeros holds. One step = "
            f"{self.u.dt * 1000:.0f} ms",
        )
        self._sid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
        self._palm = self._sid("S_grasp")
        self._tips = {f: self._sid(f"S_{f}tip") for f in FINGERS}
        self.arm = [i for i, n in enumerate(self.act_names) if n.startswith("AR")]
        self._renderer = None
        self._grip = 0.0  # the session's settle hold sends zeros = keep the targets
        self._success = False
        self.target = np.zeros(m.nu)

    # ---- core API -------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        opts = None
        if self.init_state:
            opts = {"initial_state_dict": {k: np.asarray(v, dtype=np.float64) for k, v in self.init_state.items()}}
        self.env.reset(seed=int(seed), options=opts)
        # the targets start at the current joint positions, so an all-zero action holds the hand where it is
        self.target = np.clip(self.d.qpos[self.jnt_qadr].copy(), self.lo, self.hi)
        # ...plus, for the weak arm servos, the offset that holds the arm still against gravity at the start
        import mujoco

        mujoco.mj_forward(self.m, self.d)
        for i in self.arm:
            kb = -self.m.actuator_biasprm[i, 1]  # servo stiffness in joint units (N/m or N m/rad)
            self.target[i] = np.clip(self.target[i] + self.d.qfrc_bias[self.jnt_dadr[i]] / kb, self.lo[i], self.hi[i])
        self._success = False

    def hold_action(self) -> list[float]:
        return [0.0] * self.m.nu

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=np.float64)
        if a.shape != (self.m.nu,):
            raise ValueError(f"action must have {self.m.nu} numbers")
        self.target = np.clip(self.target + a, self.lo, self.hi)
        norm = (self.target / self.k - self.u.act_mean) / self.u.act_rng  # the upstream action
        _obs, r, _t, _tr, info = self.env.step(norm)
        self._success = bool(info["success"])
        return StepInfo(success=self._success, reward=float(r))

    def success(self) -> bool:
        return self._success

    # ---- observation ----------------------------------------------------------------------------
    def palm_pos(self) -> np.ndarray:
        return self.d.site_xpos[self._palm].copy()

    def observe(self) -> dict:
        d, u = self.d, self.u
        out = {
            "palm_pos": _R(self.palm_pos()),
            "fingertips": {f: _R(d.site_xpos[s]) for f, s in self._tips.items()},
            "joints": {n: round(float(d.qpos[q]), 4) for n, q in zip(self.act_names, self.jnt_qadr, strict=False)},
            "targets": {n: round(float(t), 4) for n, t in zip(self.act_names, self.target, strict=False)},
        }
        if self.kind == "door":
            hinge = float(d.qpos[u.door_hinge_addrs])
            out.update(
                {
                    "handle_pos": _R(d.site_xpos[u.handle_site_id]),
                    "door_hinge": round(hinge, 4),
                    "latch": round(float(d.qpos[-1]), 4),
                    "door_open": hinge >= 1.35,
                }
            )
        elif self.kind == "hammer":
            nail = d.site_xpos[u.target_obj_site_id]
            goal = d.site_xpos[u.goal_site_id]
            from gymnasium_robotics.utils.rotations import quat2euler

            out.update(
                {
                    "hammer_pos": _R(d.xpos[u.obj_body_id]),
                    "hammer_rot": _R(quat2euler(d.xquat[u.obj_body_id].copy())),
                    "hammer_head_pos": _R(d.site_xpos[u.tool_site_id]),
                    "nail_pos": _R(nail),
                    "nail_goal_pos": _R(goal),
                    "nail_goal_distance": round(float(np.linalg.norm(nail - goal)), 4),
                }
            )
        elif self.kind == "pen":
            pos = d.xpos[u.obj_body_id]
            po = (d.site_xpos[u.obj_t_site_id] - d.site_xpos[u.obj_b_site_id]) / u.pen_length
            to = (d.site_xpos[u.tar_t_site_id] - d.site_xpos[u.tar_b_site_id]) / u.tar_length
            want = d.site_xpos[u.eps_ball_site_id]
            out.update(
                {
                    "pen_pos": _R(pos),
                    "pen_dir": _R(po),
                    "target_dir": _R(to),
                    "orientation_similarity": round(float(po @ to), 4),
                    "pen_home_pos": _R(want),
                    "pen_home_distance": round(float(np.linalg.norm(pos - want)), 4),
                }
            )
        else:
            ball = d.xpos[u.obj_body_id]
            tgt = d.site_xpos[u.target_obj_site_id]
            out.update(
                {
                    "ball_pos": _R(ball),
                    "target_pos": _R(tgt),
                    "ball_target_distance": round(float(np.linalg.norm(ball - tgt)), 4),
                }
            )
        return out

    def info_extra(self) -> dict:
        return {
            "actuators": {
                n: {"range": [round(float(lo), 6), round(float(hi), 6)], "unit": "m" if n.startswith("ART") else "rad"}
                for n, lo, hi in zip(self.act_names, self.lo, self.hi, strict=False)
            },
            "step_seconds": round(float(self.u.dt), 4),
            "named_skills": [
                "set NAME VALUE [NAME VALUE ...] [steps=N]: ramp those targets (absolute, rad or m) there over N steps (default 10)",
                "hand open|close|pinch [AMOUNT 0..1] [steps=N]: finger synergy (open: straight; close: power grasp; pinch: first finger and thumb)",
                "wait [N]: hold the targets for N steps (default 10)",
            ]
            + (
                [
                    "move_palm X Y Z [SPEED]: straight-line move of palm_pos with the arm slides and forearm tilt (m/s, default 0.1)"
                ]
                if self.kind == "relocate"
                else []
            ),
        }

    # ---- skills (closed-loop generators; each yielded action is one ordinary step) ---------------
    def skills(self) -> list[str]:
        return []  # no move_to/grip: the hand is driven with `robo act` and `robo skill`

    def start_skill(self, name: str, args: list[str]):
        if name == "set":
            return self._skill_set(args)
        if name == "hand":
            return self._skill_hand(args)
        if name == "move_palm":
            if not {"ARTx", "ARTy", "ARTz"} <= set(self.act_names):
                raise ValueError(
                    "move_palm needs an arm with slides (Relocate only); use `robo act` / `set` on the arm joints"
                )
            return self._skill_move_palm(args)
        if name == "wait":
            return self._skill_wait(args)
        raise ValueError(
            f"unknown skill {name!r}; skills: set, hand, wait" + (", move_palm" if self.kind == "relocate" else "")
        )

    def _index(self, n: str) -> int:
        n = n.removeprefix("A_")
        if n not in self.act_names:
            raise ValueError(f"unknown actuator {n!r}; see `robo info`")
        return self.act_names.index(n)

    def _ramp(self, goal: np.ndarray, steps: int):
        start = self.target.copy()
        goal = np.clip(goal, self.lo, self.hi)
        for k in range(1, steps + 1):
            yield list(start + (goal - start) * k / steps - self.target)

    def _skill_set(self, args):
        """set NAME VALUE [NAME VALUE ...] [steps=N]: move those targets (absolute, rad/m) there over N steps (default 10)."""
        steps, pairs = 10, []
        for a in args:
            if a.startswith("steps="):
                steps = max(1, min(100, int(a.split("=", 1)[1])))
            else:
                pairs.append(a)
        if not pairs or len(pairs) % 2:
            raise ValueError("usage: set NAME VALUE [NAME VALUE ...] [steps=N]")
        goal = self.target.copy()
        for n, v in zip(pairs[::2], pairs[1::2], strict=False):
            goal[self._index(n)] = float(v)
        yield from self._ramp(goal, steps)
        return {"targets": {n.removeprefix("A_"): round(float(self.target[self._index(n)]), 4) for n in pairs[::2]}}

    def _skill_hand(self, args):
        """hand open|close|pinch [AMOUNT] [steps=N]: finger synergy. open: all finger joints straight; close AMOUNT (0..1,
        default 1): all four fingers curl and the thumb opposes (a power grasp); pinch: thumb and first finger only."""
        if not args:
            raise ValueError("usage: hand open|close|pinch [AMOUNT 0..1] [steps=N]")
        kind, amount, steps = args[0], 1.0, 10
        for a in args[1:]:
            if a.startswith("steps="):
                steps = max(1, min(100, int(a.split("=", 1)[1])))
            else:
                amount = float(np.clip(float(a), 0.0, 1.0))
        goal = self.target.copy()
        curl = {"open": 0.0, "close": amount, "pinch": amount}.get(kind)
        if curl is None:
            raise ValueError("hand posture must be open, close or pinch")
        which = ("FF", "MF", "RF", "LF") if kind != "pinch" else ("FF",)
        # close/pinch: the finger posture of the DAPG human demonstrations' ball grasps (Relocate), scaled by AMOUNT
        curl_to = {"J2": 0.6, "J1": 1.0, "J0": 1.0}
        thumb = {"THJ4": 0.2, "THJ3": 0.9, "THJ2": 0.1, "THJ1": -0.25, "THJ0": -0.15}
        for i, n in enumerate(self.act_names):
            if kind == "open" and n[:2] in ("FF", "MF", "RF", "LF", "TH"):
                goal[i] = float(np.clip(0.0, self.lo[i], self.hi[i]))
            elif n[:2] in which and n[2:] in curl_to:
                goal[i] = curl * curl_to[n[2:]]
            elif n == "LFJ4" and "LF" in which:
                goal[i] = curl * 0.4
            elif n.startswith("TH") and kind != "open":
                goal[i] = thumb[n] * curl
        goal = np.clip(goal, self.lo, self.hi)
        yield from self._ramp(goal, steps)
        return {"posture": kind, "amount": curl}

    def _skill_move_palm(self, args):
        """move_palm X Y Z [SPEED]: move the palm point (`palm_pos`) in a straight line with the arm slides (m/s, default 0.1).
        A reference point runs along the line at SPEED; the slide targets follow it with integral action (the upstream arm
        servos are weak and sag under gravity), then the skill waits up to 50 steps for the palm to settle within 1 cm."""
        import mujoco

        if len(args) not in (3, 4):
            raise ValueError("usage: move_palm X Y Z [SPEED m/s]")
        goal = np.array([float(x) for x in args[:3]])
        speed = float(np.clip(float(args[3]) if len(args) == 4 else 0.1, 0.02, 0.3))
        # the slides, plus the forearm tilt ARRx: ARTy (vertical) cannot go below its start by much, so reaching down
        # to the table needs the tilt; a joint at its limit is left out of the solve
        slides = [i for i in self.arm if self.act_names[i].startswith("ART") or self.act_names[i] == "ARRx"]
        wts = np.array([1.0 if self.act_names[i].startswith("ART") else 0.4 for i in slides])
        jlo = np.array([self.m.jnt_range[self.m.actuator_trnid[i, 0], 0] for i in slides]) + 0.005
        jhi = np.array([self.m.jnt_range[self.m.actuator_trnid[i, 0], 1] for i in slides]) - 0.005
        qa = [self.jnt_qadr[i] for i in slides]

        def solve(J, v, q):
            free = np.ones(len(slides), dtype=bool)
            for _ in range(3):
                Jw = J * (wts * free)
                x = (wts * free) * np.linalg.lstsq(Jw, v, rcond=None)[0]
                bad = free & (((q <= jlo) & (x < 0)) | ((q >= jhi) & (x > 0)))
                if not bad.any():
                    return x
                free &= ~bad
            return x * free

        start = self.palm_pos()
        dist = float(np.linalg.norm(goal - start))
        n_move = max(1, int(np.ceil(dist / (speed * self.u.dt))))
        q_ref = self.d.qpos[qa].copy()
        integ = self.target[slides] - q_ref  # keep the current gravity offset
        ref_prev = start
        for k in range(1, n_move + 50 + 1):
            ref = start + (goal - start) * min(1.0, k / n_move)
            jp = np.zeros((3, self.m.nv))
            mujoco.mj_jacSite(self.m, self.d, jp, None, self._palm)
            J = jp[:, [self.jnt_dadr[i] for i in slides]]
            q_ref = q_ref + solve(J, ref - ref_prev, q_ref)
            ref_prev = ref
            integ += 0.01 * solve(J, ref - self.palm_pos(), self.d.qpos[qa])  # integral on the palm error (gravity sag)
            qd = self.d.qvel[[self.jnt_dadr[i] for i in slides]]
            want = np.clip(
                q_ref + integ - 0.1 * qd, self.lo[slides], self.hi[slides]
            )  # velocity damping: the servos ring
            a = np.zeros(self.m.nu)
            a[slides] = want - self.target[slides]
            yield list(a)
            if (
                k >= n_move
                and np.linalg.norm(goal - self.palm_pos()) < 0.01
                and np.linalg.norm(self.d.qvel[[self.jnt_dadr[i] for i in slides]]) < 0.02
            ):
                break
        p = self.palm_pos()
        return {
            "palm_pos": _R(p),
            "remaining_m": round(float(np.linalg.norm(goal - p)), 4),
            "reached": bool(np.linalg.norm(goal - p) < 0.015),
        }

    def _skill_wait(self, args):
        """wait [N]: hold the targets for N steps (default 10)."""
        n = max(1, min(100, int(float(args[0])) if args else 10))
        for _ in range(n):
            yield [0.0] * self.m.nu
        return {}

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
        self._renderer.update_scene(self.d, camera=self.camera)
        return self._renderer.render().copy()

    def close(self) -> None:
        try:
            if self._renderer is not None:
                self._renderer.close()
            self.env.close()
        except Exception:  # noqa: BLE001 - best-effort teardown
            pass


# ---- reference solution: replay the recorded DAPG human demonstration through the socket -------------------------------


def oracle_main(env: str) -> None:
    """env: path to oracle/demo.json ({"actions": [[upstream normalised action], ...], ...})."""
    import json

    from robouse.agent_cli import _send

    demo = json.loads(Path(env).read_text())
    info = _send({"op": "info"})["result"]
    rng = {n: v["range"] for n, v in info["actuators"].items()}
    names = info["action"]["names"]
    lo = np.array([rng[n][0] for n in names])
    hi = np.array([rng[n][1] for n in names])
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    r = _send({"op": "observe"})
    st = r["result"]["state"]
    for a in demo["actions"]:
        want = np.clip(mid + np.clip(np.asarray(a, dtype=float), -1, 1) * half, lo, hi)
        cur = np.array([st["targets"][n] for n in names])
        r = _send({"op": "act", "action": [float(x) for x in want - cur], "repeat": 1})
        if not r.get("ok") or "episode" in r["result"]:
            return
        st = r["result"]["state"]
    _send({"op": "done", "text": "oracle finished (demonstration replayed)"})
