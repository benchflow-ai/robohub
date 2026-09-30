"""Forward and inverse kinematics for real arms, from a serial chain extracted from the vendor URDF (numpy only).

A chain file (`data/<robot>_chain.json`, written by `tools/assets/urdf_to_chain.py`) lists the URDF joints in order: name, type
(fixed / revolute / prismatic), parent and child link, origin (xyz, rpy) and axis. A `Profile` (in `real/robots.py`) says how
the driver's joint values map onto the URDF joints (degrees -> radians, identity for the Metal arm and the SO-101 with
LeRobot's calibration) and where the tool point is (a link plus an offset in that link's frame).

Used by the safety envelope (points on the arm checked against the table and the workspace box before any command) and
by the backend's `move_to` skill (damped-least-squares IK on the tool point).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

import numpy as np


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF roll-pitch-yaw (extrinsic x-y-z) to a rotation matrix."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def axis_angle(axis, theta: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)


@dataclass(frozen=True)
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rot: np.ndarray
    axis: np.ndarray | None


def load_chain_file(name: str) -> dict:
    """A chain JSON shipped in robouse/real/data (by file name) or any path."""
    p = Path(name)
    if p.exists():
        return json.loads(p.read_text())
    with resources.files("robouse.real.data").joinpath(name).open() as fh:
        return json.load(fh)


class Chain:
    """Serial kinematic chain. `joint_map` maps driver joint index -> (URDF joint name, scale, offset): URDF value =
    scale * driver value + offset (scale pi/180 for degrees). `tool` is (link name, xyz offset in that link's frame)."""

    def __init__(
        self,
        chain: dict,
        joint_map: dict[int, tuple[str, float, float]],
        tool: tuple[str, list[float]],
        points: dict[str, tuple[str, list[float]]] | None = None,
    ):
        self.joints = [
            _Joint(
                j["name"],
                j["type"],
                j["parent"],
                j["child"],
                np.asarray(j["xyz"], dtype=np.float64),
                rpy_matrix(*j["rpy"]),
                None if j.get("axis") is None else np.asarray(j["axis"], dtype=np.float64),
            )
            for j in chain["joints"]
        ]
        self.joint_map = joint_map
        self.tool_link, self.tool_offset = tool[0], np.asarray(tool[1], dtype=np.float64)
        # monitored points for the workspace check: name -> (link, offset); the tool point is always monitored
        self.points = dict(points or {})
        self.n_arm = len(joint_map)

    def _values(self, q) -> dict[str, float]:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        return {name: scale * float(q[i]) + off for i, (name, scale, off) in self.joint_map.items() if i < len(q)}

    def frames(self, q) -> dict[str, np.ndarray]:
        """World 4x4 transform of every link frame."""
        vals = self._values(q)
        out: dict[str, np.ndarray] = {}
        for j in self.joints:
            parent = out.get(j.parent, np.eye(4))
            local = np.eye(4)
            local[:3, :3] = j.rot
            local[:3, 3] = j.xyz
            motion = np.eye(4)
            if j.axis is not None and j.name in vals:
                if j.type in ("revolute", "continuous"):
                    motion[:3, :3] = axis_angle(j.axis, vals[j.name])
                elif j.type == "prismatic":
                    motion[:3, 3] = j.axis / np.linalg.norm(j.axis) * vals[j.name]
            out[j.child] = parent @ local @ motion
        return out

    def point(self, frames: dict[str, np.ndarray], link: str, offset) -> np.ndarray:
        T = frames[link]
        return T[:3, :3] @ np.asarray(offset, dtype=np.float64) + T[:3, 3]

    def tool(self, q) -> np.ndarray:
        """World position (m) of the tool point (between the fingertips)."""
        return self.point(self.frames(q), self.tool_link, self.tool_offset)

    def tool_axis(self, q) -> np.ndarray:
        """Unit direction from the tool link's origin to the tool point (the approach direction)."""
        T = self.frames(q)[self.tool_link]
        d = T[:3, :3] @ self.tool_offset
        n = float(np.linalg.norm(d))
        return d / n if n > 0 else T[:3, 0]

    def monitored(self, q) -> dict[str, np.ndarray]:
        f = self.frames(q)
        pts = {name: self.point(f, link, off) for name, (link, off) in self.points.items()}
        pts["tool"] = self.point(f, self.tool_link, self.tool_offset)
        return pts

    # ---- inverse kinematics (position of the tool point) ------------------------------------------------------------
    def jacobian(self, q, eps: float = 0.05) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).copy()
        base = self.tool(q)
        J = np.zeros((3, self.n_arm))
        for i in range(self.n_arm):
            dq = q.copy()
            dq[i] += eps
            J[:, i] = (self.tool(dq) - base) / eps
        return J

    def solve(
        self,
        target,
        q0,
        low,
        high,
        *,
        seeds=(),
        iters: int = 200,
        damping: float = 0.02,
        step: float = 6.0,
        tol: float = 1e-3,
    ) -> tuple[np.ndarray, float]:
        """Damped-least-squares IK for the tool point; tries q0 then each seed; returns (q, residual in m).
        Joint values beyond the arm joints (e.g. the gripper) are passed through from q0."""
        target = np.asarray(target, dtype=np.float64).reshape(3)
        low, high = np.asarray(low, dtype=np.float64), np.asarray(high, dtype=np.float64)
        best_q, best_e = np.asarray(q0, dtype=np.float64).copy(), float("inf")
        starts = [np.asarray(q0, dtype=np.float64)]
        for s in seeds:
            qs = np.asarray(q0, dtype=np.float64).copy()
            qs[: len(s)] = s
            starts.append(qs)
        for q_start in starts:
            q = q_start.copy()
            n = self.n_arm
            q[:n] = np.clip(q[:n], low[:n], high[:n])
            for _ in range(iters):
                err = target - self.tool(q)
                d = float(np.linalg.norm(err))
                if d < best_e:
                    best_q, best_e = q.copy(), d
                if d < tol:
                    break
                J = self.jacobian(q)
                dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), err)
                nrm = float(np.linalg.norm(dq))
                if nrm > step:
                    dq *= step / nrm
                q[:n] = np.clip(q[:n] + dq, low[:n], high[:n])
            if best_e < tol:
                break
        return best_q, best_e


def urdf_to_chain(urdf_path: str | Path, robot: str, source: str = "") -> dict:
    """Parse a URDF into the chain JSON layout (joints in parent-to-child order)."""
    import xml.etree.ElementTree as ET

    root = ET.parse(str(urdf_path)).getroot()
    joints = []
    for j in root.findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        joints.append(
            {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": xyz,
                "rpy": rpy,
                "axis": [float(v) for v in a.get("xyz").split()]
                if a is not None and j.get("type") != "fixed"
                else None,
                "limit": {"lower": float(lim.get("lower")), "upper": float(lim.get("upper"))}
                if lim is not None and lim.get("lower") is not None
                else None,
            }
        )
    # order parent before child
    by_parent: dict[str, list[dict]] = {}
    for j in joints:
        by_parent.setdefault(j["parent"], []).append(j)
    children = {j["child"] for j in joints}
    roots = [p for p in by_parent if p not in children]
    ordered, stack = [], list(roots)
    while stack:
        link = stack.pop(0)
        for j in by_parent.get(link, []):
            ordered.append(j)
            stack.append(j["child"])
    return {
        "robot": robot,
        "sources": {"urdf": source or Path(urdf_path).name},
        "joint_mapping": "identity: degrees -> radians",
        "joints": ordered,
        "links": [{"name": l.get("name")} for l in root.findall("link")],
    }
