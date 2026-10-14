"""Physical bookkeeping over an episode (copied from the robotics-tasks-20260917 episode runner; simulator side only)."""

import math

import mujoco
import numpy as np


class Monitor:
    """Physical bookkeeping over an episode; all of it is simulator side."""

    def __init__(self, sim, task, scene):
        model = sim.model
        self.sim = sim
        self.task = task
        self.scene = scene
        body_names = [model.body(i).name for i in range(model.nbody)]
        self.robot_bodies = {i for i, n in enumerate(body_names) if n.startswith(("left/", "right/"))}
        self.arm_of_body = {i: n.split("/")[0] for i, n in enumerate(body_names) if n.startswith(("left/", "right/"))}
        geom_names = [model.geom(i).name for i in range(model.ngeom)]
        self.desk_geoms = {i for i, n in enumerate(geom_names) if n.startswith(tuple(task.DESK_PREFIXES))}
        self.object_bodies = {name: model.body(name).id for name in task.observation_bodies(scene)}
        self.body_of_geom = model.geom_bodyid
        pairs = getattr(task, "FORCE_PAIRS", None)
        if pairs:
            self.force_a = {i for i, n in enumerate(geom_names) if n.startswith(tuple(pairs[0]))}
            self.force_b = {i for i, n in enumerate(geom_names) if n.startswith(tuple(pairs[1]))}
        else:
            self.force_a = self.force_b = set()
        self.desk_contact_samples = 0
        self.samples = 0
        self.peak_force_n = 0.0
        self.joint_limit_steps = 0
        self.control_steps = 0
        self.touch_history = {name: set() for name in self.object_bodies}
        self.current_touch = {name: set() for name in self.object_bodies}
        self.last_touch = {name: set() for name in self.object_bodies}
        self.settle_start = None
        self.min_object_z = {name: math.inf for name in self.object_bodies}
        self._force = np.zeros(6)

    def substep(self, sim):
        self.samples += 1
        data = sim.data
        count = data.ncon
        if not count:
            return
        geoms = data.contact.geom[:count]
        bodies = self.body_of_geom[geoms]
        for index in range(count):
            first, second = int(geoms[index, 0]), int(geoms[index, 1])
            robot = (int(bodies[index, 0]) in self.robot_bodies, int(bodies[index, 1]) in self.robot_bodies)
            if any(robot) and (first in self.desk_geoms or second in self.desk_geoms):
                self.desk_contact_samples += 1
            if self.force_a and (
                (first in self.force_a and second in self.force_b) or (second in self.force_a and first in self.force_b)
            ):
                mujoco.mj_contactForce(sim.model, data, index, self._force)
                self.peak_force_n = max(self.peak_force_n, float(abs(self._force[0])))

    def control(self, sim):
        self.control_steps += 1
        data = sim.data
        at_limit = False
        for side in sim.scene.arms:
            q = sim.arm_joint_positions(side)
            limits = sim.joint_limits[side]
            span = limits[:, 1] - limits[:, 0]
            # Full-turn joints have no meaningful limit; +pi and -pi are the same pose.
            bounded = span < 6.0
            if np.any((q <= limits[:, 0] + 0.01 * span)[bounded]) or np.any((q >= limits[:, 1] - 0.01 * span)[bounded]):
                at_limit = True
        self.joint_limit_steps += int(at_limit)
        count = data.ncon
        touching = {name: set() for name in self.object_bodies}
        if count:
            geoms = data.contact.geom[:count]
            bodies = self.body_of_geom[geoms]
            lookup = {bid: name for name, bid in self.object_bodies.items()}
            for index in range(count):
                first, second = int(bodies[index, 0]), int(bodies[index, 1])
                for a, b in ((first, second), (second, first)):
                    if a in lookup and b in self.arm_of_body:
                        touching[lookup[a]].add(self.arm_of_body[b])
        for name, arms in touching.items():
            self.touch_history[name] |= arms
            if arms:
                self.last_touch[name] = set(arms)
            self.min_object_z[name] = min(self.min_object_z[name], float(data.body(name).xpos[2]))
        self.current_touch = touching
        return touching

    def mark_settle_start(self):
        self.settle_start = {
            name: (self.sim.data.body(name).xpos.copy(), self.sim.data.body(name).xquat.copy())
            for name in self.object_bodies
        }

    def summary(self):
        return {
            "desk_contact_fraction": self.desk_contact_samples / max(1, self.samples),
            "last_arm_to_touch": {k: sorted(v) for k, v in self.last_touch.items()},
            "desk_contact_samples": self.desk_contact_samples,
            "peak_contact_force_n": self.peak_force_n,
            "joint_limit_fraction": self.joint_limit_steps / max(1, self.control_steps),
            "arms_that_touched": {k: sorted(v) for k, v in self.touch_history.items()},
            "min_object_z_m": {k: (None if v == math.inf else v) for k, v in self.min_object_z.items()},
        }
