"""Robo Use replacement for capx/integrations/motion/pyroki.py (CaP-X 53e9966, MIT).

CaP-X's `init_pyroki()` returns an IK function that posts to its PyRoKi server (capx/serving/launch_pyroki_server.py,
`/ik`). This version runs the same solve in-process, with the same robot model and solver: `panda_description` from
robot_descriptions with 0.15 rad kept clear of every revolute joint limit, target link `panda_hand`, PyRoKi's
`solve_ik` for the first solve and `solve_ik_vel_cost` when a previous configuration is given (the server's
`_do_solve_ik`). CaP-X's `--profile minimal` runs this server on the CPU as well.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np


def set_min_distance_from_limits(urdf, min_distance_from_limits: float = 0.15):
    for joint in urdf.robot.joints:
        if joint.type == "revolute" and joint.limit is not None:
            if joint.limit.lower is not None and joint.limit.upper is not None:
                joint.limit.lower = joint.limit.lower + min_distance_from_limits
                joint.limit.upper = joint.limit.upper - min_distance_from_limits
    return urdf


@lru_cache(maxsize=1)
def _robot():
    import pyroki as pk  # type: ignore
    from robot_descriptions.loaders.yourdfpy import load_robot_description

    return pk.Robot.from_urdf(set_min_distance_from_limits(load_robot_description("panda_description")))


def init_pyroki(server_url: str = "", target_link_name: str = "panda_hand"):
    import capx.integrations.motion.pyroki_snippets as pks

    robot = _robot()

    def ik_solve_fn(target_pose_wxyz_xyz: np.ndarray, prev_cfg: np.ndarray | None = None) -> np.ndarray:
        t = np.asarray(target_pose_wxyz_xyz, dtype=np.float64)
        if prev_cfg is None:
            q = pks.solve_ik(robot=robot, target_link_name=target_link_name, target_position=t[-3:], target_wxyz=t[:-3])
        else:
            q = pks.solve_ik_vel_cost(
                robot=robot,
                target_link_name=target_link_name,
                target_position=t[-3:],
                target_wxyz=t[:-3],
                prev_cfg=np.asarray(prev_cfg, dtype=np.float64),
            )
        return np.asarray(q, dtype=np.float32)

    return ik_solve_fn
