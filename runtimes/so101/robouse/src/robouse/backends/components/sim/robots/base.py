"""What every embodiment driver shares: the physics and control clocks, the Driver base class and floor robots."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ....embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of


PHYS_DT = .002
CONTROL_DT = .05
N_SUB = round(CONTROL_DT / PHYS_DT)

GRASP_DOC = {
    "top": "top-down: the gripper points straight down and its fingers close along the world y axis",
    "front": "front: the gripper points along the robot's heading and stays level; its fingers close horizontally",
}


def _yawq(yaw: float) -> list[float]:
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _mulq(a, b) -> np.ndarray:
    import mujoco

    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


class Driver:
    key = ""
    name = ""
    kind = ""            # BenchFlow embodiment kind
    family = ""          # robouse embodiment family
    mount = ""           # "arm" | "bimanual" | "floor" | "airspace"
    caps: tuple = ()
    grasp_mode = None    # "top" | "front" | None
    max_grip = 0.0       # widest object the fingers can close on with a margin (m)
    assets: list = []
    arms: list = []
    prefix = "robot/"

    def __init__(self, layout, params: dict | None = None):
        self.layout = layout
        self.params = dict(params or {})
        self.m = self.d = None
        self.robot_geoms: set = set()
        self.finger_sides: dict = {}   # arm -> (geoms of one finger, geoms of the other)
        self.body_geoms: set = set()   # geoms that must not hit furniture hard (base, legs, torso)

    # ---- declaration -------------------------------------------------------------------------------------------
    def groups(self) -> list[ActionGroup]:
        raise NotImplementedError

    def skills(self) -> list[Skill]:
        raise NotImplementedError

    def cameras(self) -> list[str]:
        return []

    def inspect_camera(self) -> str | None:
        return None

    def vec(self, **kw) -> list[float]:
        """A full action vector: groups named in kw (dots -> underscores) get those values, the rest hold."""
        out: list[float] = []
        for g in self.groups():
            v = kw.get(g.name.replace(".", "_"))
            if v is None:
                out += list(g.hold) if g.hold is not None else [0.0] * len(g.names)
            else:
                v = list(np.atleast_1d(np.asarray(v, float)))
                out += [float(np.clip(x, lo, hi)) for x, lo, hi in zip(v, g.low, g.high)]
        return out

    def hold(self) -> list[float]:
        return self.vec()

    # ---- building ----------------------------------------------------------------------------------------------
    def options(self, s) -> None:
        import mujoco

        o = s.option
        o.timestep = PHYS_DT
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        o.impratio = 10
        o.noslip_iterations = 3
        o.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD

    def attach(self, s) -> None:
        raise NotImplementedError

    def bind(self, m, d) -> None:
        import mujoco

        self.m, self.d = m, d
        pre = self.prefix
        self.robot_bodies = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(pre)}
        self.robot_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in self.robot_bodies}

    def bodies_named(self, names) -> set[int]:
        return {self.m.body(self.prefix + n).id for n in names}

    def geoms_of(self, bodies: set[int]) -> set[int]:
        return {g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] in bodies}

    def init_state(self) -> None:
        pass

    def post_forward(self) -> None:
        pass

    # ---- running -----------------------------------------------------------------------------------------------
    def control(self, a: np.ndarray) -> None:
        raise NotImplementedError

    def substep(self, dt: float) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def public(self) -> dict:
        """What the robot is and can do (part of `robo observe`)."""
        return {"embodiment": self.key, "name": self.name, "capabilities": list(self.caps),
                **({"grasp": GRASP_DOC[self.grasp_mode]} if self.grasp_mode else {})}

    # arm and base helpers used by skills (overridden where they apply)
    def ee_pos(self, arm: str) -> np.ndarray:
        raise NotImplementedError

    def base_pose(self) -> tuple[float, float, float]:
        raise NotImplementedError


def _delete(spec, el) -> None:
    if hasattr(spec, "delete"):
        spec.delete(el)
    else:
        el.delete()


# ======================================================================================================================
# floor robots
# ======================================================================================================================

class FloorRobot(Driver):
    mount = "floor"

    def start_pose(self) -> tuple[float, float, float]:
        x, y, yaw = self.layout.mounts["floor"]["start"]
        return x, y, math.radians(yaw)
