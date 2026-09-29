"""BEHAVIOR-1K simulation worker. Runs on an NVIDIA RTX GPU machine inside the BEHAVIOR-1K conda environment
(OmniGibson 3.9 on Isaac Sim 5.1), never on the robouse host.

One worker process owns one Isaac Sim instance with one BEHAVIOR activity loaded (scene + task instance + R1Pro
robot). It answers JSON-over-HTTP requests from the robouse `behavior` backend (behavior.py):

  POST /rpc  {"method": "...", "args": {...}}  ->  {"ok": true, "result": ...} | {"ok": false, "error": "..."}

Methods: info, reset, observe, skill, wait, render, success, goal_status. Images travel as base64 JPEG.
Requests carry the shared secret from $ROBOUSE_BEHAVIOR_SECRET in the `X-Robouse-Secret` header.

Robot actions are OmniGibson's symbolic semantic action primitives (omnigibson.action_primitives.
symbolic_semantic_action_primitives) for open, close, toggle_on and toggle_off: they set the post-condition state
directly (no arm motion planning), then let physics settle. The worker implements the rest itself and adds what the
symbolic set leaves out, so the agent still has to plan the household task:
  - navigate_to OBJ: teleport the robot base to a free spot next to OBJ (traversability map), facing it.
  - grasp OBJ: a kinematic carry. The object's gravity is switched off and it rides 0.45 m in front of the base at
    1.1 m height until it is placed or released. (Sticky/assisted grasp joints kept dropping objects during
    teleport-based navigation.) Task objects resting inside or on top of the grasped object (food on a plate, items in
    a box) are carried along at their relative pose, as a tray or a box carries its contents.
  - place_on_top / place_inside OBJ: release, then BEHAVIOR's own kinematic sampler (object_states OnTop / Inside
    set_value) puts the object in the relation; carried contents are put back at their relative pose; physics settles;
    the relation is re-checked.
  - reach: grasp/place/open/close/toggle fail unless the robot is within REACH_M of the target (navigate first).
  - grasp fails on objects that are fixed, too big, or inside a closed container; the hand holds one object, and
    open/close/toggle need an empty hand.
  - place_inside fails if the container is openable and closed.
Success is BEHAVIOR's own BDDL goal check (the task's PredicateGoal termination condition).

Task instances: instance 0 is the challenge's cached activity scene (`*_0_0_template.json`). Any other instance k loads
that scene and then applies the challenge's instance file `*_0_k_template-tro_state.json` (task-relevant object states
and the robot's start pose) the way BEHAVIOR's challenge evaluator does, and makes that the state `reset` restores.

Usage (on the GPU machine):
  OMNIGIBSON_HEADLESS=1 python behavior_worker.py --activity turning_on_radio --scene house_double_floor_lower \
    --instance 0 --port 8765
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import sys
import time
import traceback
import types
from http.server import BaseHTTPRequestHandler, HTTPServer

REACH_M = 1.25          # max horizontal distance from the robot base to a target object's bounding box
NAV_OFFSETS = (0.35, 0.5, 0.7, 0.9, 1.1)   # stand-off distances (m) from the target's bounding box tried by navigate_to
ROBOT_RADIUS_M = 0.38   # clearance checked on the traversability map around a candidate base position
PLACE_ATTEMPTS = 4     # placement sampler draws before place_on_top / place_inside reports failure
PLACE_BUDGET_S = 60    # ... or fewer, once the draws have taken this long
MAX_GRASP_EXTENT_M = float(os.environ.get("ROBOUSE_BEHAVIOR_MAX_GRASP_M", "2.5"))
FRAME_EVERY = 3         # during a skill, keep one video frame every N env steps
MAX_FRAMES_PER_SKILL = 12
IMG_W, IMG_H = 640, 480

SKILLS = {
    "navigate_to": "navigate_to OBJ - drive the base next to OBJ, facing it (carries a held object)",
    "grasp": "grasp OBJ - pick up OBJ with the gripper (hand must be empty, OBJ within reach)",
    "place_on_top": "place_on_top OBJ - put the held object on top of OBJ",
    "place_inside": "place_inside OBJ - put the held object inside OBJ (open it first if it opens)",
    "open": "open OBJ - open a door/drawer/lid of OBJ (hand must be empty)",
    "close": "close OBJ - close OBJ (hand must be empty)",
    "toggle_on": "toggle_on OBJ - switch OBJ on (hand must be empty)",
    "toggle_off": "toggle_off OBJ - switch OBJ off (hand must be empty)",
    "release": "release - drop the held object where the hand is",
}


class SkillError(Exception):
    pass


def _short_settle(prims):
    """Shorter settle than OmniGibson's (50 fixed steps, then up to MAX_STEPS_FOR_SETTLING): hold the joints for 8-30
    steps until the base stops moving. Each primitive settles once or twice, and every simulator step in a full
    BEHAVIOR house costs about 0.3 s, so the stock settle made a single grasp take close to a minute."""
    import torch as th

    for i in range(30):
        yield prims._postprocess_action(prims.robot.q_to_action(prims.robot.get_joint_positions()))
        if i >= 8 and th.norm(prims.robot.get_linear_velocity()) < 0.01:
            break


class Worker:
    def __init__(self, activity: str, instance: int, scene_model: str, rooms: list[str] | None):
        import torch as th

        # Several workers share one machine; torch's default (one thread per core) oversubscribed the CPUs so badly that
        # a skill took minutes when five workers were busy at once.
        th.set_num_threads(int(os.environ.get("ROBOUSE_BEHAVIOR_TORCH_THREADS", "8")))
        import omnigibson as og
        from omnigibson.macros import gm

        gm.ENABLE_OBJECT_STATES = True
        gm.USE_GPU_DYNAMICS = False
        gm.ENABLE_TRANSITION_RULES = False
        gm.ENABLE_FLATCACHE = True
        self.og = og
        self.activity = activity
        self.instance = instance
        import yaml

        robot_cfg = yaml.safe_load(open(os.path.join(og.example_config_path, "r1pro_primitives.yaml")))["robots"][0]
        robot_cfg["obs_modalities"] = ["rgb"]
        robot_cfg["include_sensor_names"] = None
        robot_cfg["grasping_mode"] = "sticky"
        robot_cfg.setdefault("sensor_config", {}).setdefault("VisionSensor", {})["sensor_kwargs"] = {
            "image_height": IMG_H, "image_width": IMG_W}
        cfg = {
            "env": {"action_frequency": 30, "rendering_frequency": 30, "physics_frequency": 120},
            "render": {"viewer_width": IMG_W, "viewer_height": IMG_H},
            "scene": {"type": "InteractiveTraversableScene", "scene_model": scene_model, "trav_map_with_objects": True,
                      "load_room_types": None, "load_room_instances": rooms, "include_robots": False},
            "robots": [robot_cfg],
            "task": {"type": "BehaviorTask", "activity_name": activity, "activity_definition_id": 0,
                     "activity_instance_id": 0, "online_object_sampling": False,
                     "use_presampled_robot_pose": True, "highlight_task_relevant_objects": False,
                     "termination_config": {"max_steps": 100000}, "include_obs": False},
        }
        t = time.time()
        self.env = og.Environment(configs=cfg)
        # Render only when an image is requested (_follow_rgb / _head_rgb), not on every physics step: several workers
        # share one GPU, and per-step RTX rendering made each skill take minutes.
        og.sim._render_on_step = False
        self.scene = self.env.scenes[0]
        self.robot = self.scene.robots[0]
        from omnigibson.action_primitives.symbolic_semantic_action_primitives import (
            SymbolicSemanticActionPrimitives, SymbolicSemanticActionPrimitiveSet)

        self.P = SymbolicSemanticActionPrimitiveSet
        self.prims = SymbolicSemanticActionPrimitives(self.env, self.robot)
        self.prims._settle_robot = types.MethodType(_short_settle, self.prims)
        self._frames: list[str] = []
        self.cam = og.sim.viewer_camera
        self.cam.add_modality("rgb")
        self.scope_names = [n for n, e in self.env.task.object_scopes[0].items()
                            if e is not None and not n.startswith("agent") and hasattr(e, "aabb")]
        if instance:
            self._load_instance(scene_model, activity, instance)
        self.env.reset()
        self._settle(30)
        self.load_s = round(time.time() - t, 1)

    def _load_instance(self, scene_model: str, activity: str, instance: int) -> None:
        """Apply the challenge's task-instance file (task-relevant object states + robot start pose) on top of the
        cached instance-0 scene, as omnigibson.eval.evaluator does, and make it the state that reset() restores."""
        from omnigibson.utils.python_utils import recursively_convert_to_torch

        root = os.path.join(self.og.macros.gm.DATA_PATH, "2026-challenge-task-instances", "scenes", scene_model, "json",
                            f"{scene_model}_task_{activity}_instances",
                            f"{scene_model}_task_{activity}_0_{instance}_template-tro_state.json")
        with open(root) as f:
            tro = recursively_convert_to_torch(json.load(f))
        scope = self.env.task.object_scopes[0]
        self.robot.reset()
        for key, state in tro.items():
            if key == "robot_poses":
                poses = {k.lower(): v for k, v in state.items()}
                pose = (poses.get("robot") or poses[self.robot.model.lower()])[0]
                self.robot.set_position_orientation(pose["position"], pose["orientation"], frame="scene")
                self.scene.write_task_metadata(key=key, data=state)
            elif scope.get(key) is not None:
                scope[key].load_state(state, serialized=False)
        self.og.sim.update_handles() if hasattr(self.og.sim, "update_handles") else None
        for _ in range(25):
            self.og.sim.step_physics()
            for n, e in scope.items():
                if e is not None and n in self.scope_names:
                    e.keep_still()
        self.scene.update_initial_file()

    # ---- helpers ---------------------------------------------------------------------------------------------------
    def _scope(self) -> dict:
        return {n: e for n, e in self.env.task.object_scopes[0].items() if n in self.scope_names}

    def _obj(self, name: str):
        e = self._scope().get(name)
        if e is None:
            raise SkillError(f"unknown object {name!r}; objects: {', '.join(self.scope_names)}")
        return e

    def _empty_action(self):
        return self.prims._empty_action() if hasattr(self.prims, "_empty_action") else \
            self.prims._postprocess_action(self.robot.q_to_action(self.robot.get_joint_positions()))

    def _step(self, action, record: bool = True):
        # robot action + one simulator step, without env.step's observation, reward and termination bookkeeping
        # (the BDDL goal is checked only when the episode is judged)
        import torch as th

        self._hold_carried()
        self.robot.apply_action(th.as_tensor(action, dtype=th.float32))
        self.og.sim.step()
        self._n = getattr(self, "_n", 0) + 1
        if record and self._n % FRAME_EVERY == 0 and len(self._frames) < MAX_FRAMES_PER_SKILL:
            self._frames.append(self._jpeg(self._follow_rgb()))

    def _settle(self, n: int, record: bool = False):
        for _ in range(n):
            self._step(self._empty_action(), record)

    def _run(self, gen):
        for action in gen:
            if action is None:
                break
            self._step(action)

    def _robot_xy_yaw(self):
        import torch as th

        pos, quat = self.robot.get_position_orientation()
        x, y, z, w = [float(v) for v in quat]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), yaw

    def _held(self):
        return getattr(self, "_carry", None)

    def _carry_pose(self):
        """Where a carried object rides: 0.45 m in front of the base at 1.1 m height (in front of the torso)."""
        import torch as th

        x, y, yaw = self._robot_xy_yaw()
        return th.tensor([x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw), 1.1], dtype=th.float32)

    def _hold_carried(self):
        import torch as th

        obj = self._held()
        if obj is None:
            return
        obj.set_position_orientation(position=self._carry_pose())
        obj.set_linear_velocity(th.zeros(3))
        obj.set_angular_velocity(th.zeros(3))
        self._pose_riders(obj)

    def _riders_of(self, obj) -> list:
        """Movable task objects resting inside or on top of obj, transitively (a pizza on a plate, items in a box)."""
        from omnigibson import object_states as S

        out, frontier = [], [obj]
        movable = [e for e in self._scope().values() if not getattr(e, "fixed_base", False) and e is not obj]
        while frontier:
            base = frontier.pop()
            for e in movable:
                if e in out:
                    continue
                try:
                    on = (S.Inside in e.states and e.states[S.Inside].get_value(base)) or \
                         (S.OnTop in e.states and e.states[S.OnTop].get_value(base))
                except Exception:
                    on = False
                if on:
                    out.append(e)
                    frontier.append(e)
        return out

    def _pose_riders(self, obj):
        """Keep carried contents at their pose relative to obj."""
        import torch as th
        import omnigibson.utils.transform_utils as T

        riders = getattr(self, "_riders", [])
        if not riders:
            return
        bp, bq = obj.get_position_orientation()
        for e, (rp, rq) in riders:
            p, q = T.pose_transform(bp, bq, rp, rq)
            e.set_position_orientation(position=p, orientation=q)
            e.set_linear_velocity(th.zeros(3))
            e.set_angular_velocity(th.zeros(3))

    def _pick(self, obj):
        import omnigibson.utils.transform_utils as T

        riders = self._riders_of(obj)
        bp, bq = obj.get_position_orientation()
        inv_p, inv_q = T.invert_pose_transform(bp, bq)
        self._riders = []
        for e in riders:
            p, q = e.get_position_orientation()
            self._riders.append((e, T.pose_transform(inv_p, inv_q, p, q)))
            try:
                e.disable_gravity()
            except Exception:
                pass
        self._carry = obj
        try:
            obj.disable_gravity()
        except Exception:
            pass
        self._hold_carried()
        self._settle(6, record=True)

    def _drop(self):
        obj, self._carry = self._held(), None
        riders, self._riders = getattr(self, "_riders", []), []
        for e in [obj] + [r[0] for r in riders]:
            if e is not None:
                try:
                    e.enable_gravity()
                except Exception:
                    pass
        return obj

    def _box_dist_xy(self, obj) -> float:
        """Horizontal distance from the robot base to obj's axis-aligned bounding box (0 if inside it)."""
        x, y, _ = self._robot_xy_yaw()
        lo, hi = obj.aabb
        dx = max(float(lo[0]) - x, 0.0, x - float(hi[0]))
        dy = max(float(lo[1]) - y, 0.0, y - float(hi[1]))
        return math.hypot(dx, dy)

    def _room_of_point(self, xy):
        import torch as th

        try:
            return self.scene._seg_map.get_room_instance_by_point(th.tensor(xy[:2], dtype=th.float32))
        except Exception:
            return None

    def _room(self, obj) -> str | None:
        """The room obj is in now. `in_rooms` is scene metadata, so it holds only for fixed objects (a movable
        object keeps the room it was loaded in)."""
        rooms = getattr(obj, "in_rooms", None)
        if rooms and getattr(obj, "fixed_base", False):
            return rooms[0]
        return self._room_of_point([float(v) for v in obj.aabb_center[:2]]) or (rooms[0] if rooms else None)

    def _free(self, xy, floor: int = 0) -> bool:
        import torch as th

        tm = self.scene._trav_map
        fm = tm.floor_map[floor]
        r = max(1, int(math.ceil(ROBOT_RADIUS_M / tm.map_resolution)))
        c = tm.world_to_map(th.tensor(xy, dtype=th.float32))
        i, j = int(c[0]), int(c[1])
        h, w = fm.shape[0], fm.shape[1]
        if i - r < 0 or j - r < 0 or i + r >= h or j + r >= w:
            return False
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r and int(fm[i + di, j + dj]) == 0:
                    return False
        return True

    # ---- skills ----------------------------------------------------------------------------------------------------
    def _check_reach(self, obj):
        d = self._box_dist_xy(obj)
        if d > REACH_M:
            raise SkillError(f"{self._name(obj)} is {d:.2f} m away (reach {REACH_M} m); navigate_to it first")

    def _name(self, obj) -> str:
        for n, e in self._scope().items():
            if e is obj:
                return n
        return getattr(obj, "name", "?")

    def _closed_container_of(self, obj):
        from omnigibson import object_states as S

        for n, c in self._scope().items():
            if c is obj or S.Open not in getattr(c, "states", {}):
                continue
            try:
                if S.Inside in obj.states and obj.states[S.Inside].get_value(c) and not c.states[S.Open].get_value():
                    return n
            except Exception:
                continue
        return None

    def navigate_to(self, obj):
        import torch as th
        from omnigibson.utils.transform_utils import euler2quat

        held = self._held()
        center = [float(v) for v in obj.aabb_center]
        lo, hi = obj.aabb
        half = [(float(hi[0]) - float(lo[0])) / 2, (float(hi[1]) - float(lo[1])) / 2]
        target_room = self._room(obj)
        rx, ry, _ = self._robot_xy_yaw()
        cands = []
        if min(half) > 1.5:  # a floor, a lawn or another room-sized object: stand on it, in its room, nearest first
            step = 0.5
            nx, ny = int(2 * half[0] / step), int(2 * half[1] / step)
            for i in range(1, nx):
                for j in range(1, ny):
                    p = [float(lo[0]) + i * step, float(lo[1]) + j * step]
                    if self._free(p) and (not target_room or self._room_of_point(p) == target_room):
                        cands.append((0.0, math.hypot(p[0] - rx, p[1] - ry), p))
            if cands:
                cands.sort(key=lambda c: c[1])
                p = cands[0][2]
                z = float(self.robot.get_position_orientation()[0][2])
                yaw = math.atan2(p[1] - ry, p[0] - rx)
                self.robot.set_position_orientation(th.tensor([p[0], p[1], z], dtype=th.float32),
                                                    euler2quat(th.tensor([0.0, 0.0, yaw], dtype=th.float32)))
                self._settle(10, record=True)
                return
        for off in NAV_OFFSETS:
            for k in range(32):
                a = 2 * math.pi * k / 32
                dx, dy = math.cos(a), math.sin(a)
                # distance from the box centre to its boundary along (dx, dy), then the stand-off
                t = min(half[0] / abs(dx) if abs(dx) > 1e-6 else 1e9, half[1] / abs(dy) if abs(dy) > 1e-6 else 1e9)
                p = [center[0] + (t + off) * dx, center[1] + (t + off) * dy]
                if not self._free(p):
                    continue
                if target_room and self._room_of_point(p) not in (target_room, None):
                    continue
                cands.append((off, math.hypot(p[0] - rx, p[1] - ry), p))
            if cands:
                break
        if not cands:
            raise SkillError(f"no free spot found next to {self._name(obj)}")
        cands.sort(key=lambda c: c[1])
        p = cands[0][2]
        yaw = math.atan2(center[1] - p[1], center[0] - p[0])
        z = float(self.robot.get_position_orientation()[0][2])
        self.robot.set_position_orientation(th.tensor([p[0], p[1], z], dtype=th.float32),
                                            euler2quat(th.tensor([0.0, 0.0, yaw], dtype=th.float32)))
        self._settle(10, record=True)

    def _place(self, target, target_name: str, how: str) -> dict:
        """Release the held object and put it in the goal relation with BEHAVIOR's own kinematic sampler
        (object_states.OnTop / Inside set_value), then let physics settle. The symbolic primitive's sampler, which runs
        while the object is still attached to the hand, failed on open containers such as the trash can."""
        from omnigibson import object_states as S

        held = self._held()
        pred = S.Inside if how == "place_inside" else S.OnTop
        held_name = self._name(held)
        riders = list(getattr(self, "_riders", []))
        self._drop()
        ok, t0 = False, time.time()
        for attempt in range(PLACE_ATTEMPTS):  # the sampler is random: a failed or unstable spot gets another draw
            if attempt and time.time() - t0 > PLACE_BUDGET_S:  # keep a skill well inside the robo client's timeout
                break
            if pred is S.OnTop and attempt == 0 and self._room_sized(target):
                # putting something down on a floor or lawn: set it down in front of the robot, not at a random spot
                if not self._set_down_in_front(held, target):
                    continue
            elif pred is S.OnTop and attempt == 0 and not getattr(target, "fixed_base", False):
                self._set_down_centred(held, target)  # stacking on a movable object: centred on its top
            elif not held.states[pred].set_value(target, True):
                continue
            if riders:  # carried contents go back to their pose relative to the placed object
                self._riders = riders
                self._pose_riders(held)
                self._riders = []
            self._settle(20, record=True)
            ok = bool(held.states[pred].get_value(target))
            if ok:
                break
        if not ok:
            where = "inside" if how == "place_inside" else "on top of"
            raise SkillError(f"could not place {held_name} {where} {target_name} (no free spot found or it fell off); "
                             f"it was released and is no longer held")
        return {"message": f"{how} {target_name}: ok ({held_name})"}

    @staticmethod
    def _room_sized(obj) -> bool:
        lo, hi = obj.aabb
        return min(float(hi[0]) - float(lo[0]), float(hi[1]) - float(lo[1])) > 3.0

    def _set_down_in_front(self, held, target) -> bool:
        """Put `held` on the floor-like `target` 0.5 m beyond its own half-length in front of the base, if that spot
        is free on the traversability map and lies over the target."""
        import torch as th

        x, y, yaw = self._robot_xy_yaw()
        lo, hi = held.aabb
        half = max(float(hi[0]) - float(lo[0]), float(hi[1]) - float(lo[1])) / 2
        pos = held.get_position_orientation()[0]
        base_to_bottom = float(pos[2]) - float(lo[2])
        tlo, thi = target.aabb
        for d in (0.5 + half, 0.8 + half, 0.3 + half):
            px, py = x + d * math.cos(yaw), y + d * math.sin(yaw)
            if not (float(tlo[0]) <= px <= float(thi[0]) and float(tlo[1]) <= py <= float(thi[1])):
                continue
            held.set_position_orientation(position=th.tensor([px, py, float(thi[2]) + base_to_bottom + 0.01],
                                                             dtype=th.float32))
            return True
        return False

    def _set_down_centred(self, held, target) -> None:
        import torch as th

        lo, _ = held.aabb
        pos = held.get_position_orientation()[0]
        c, (_, thi) = target.aabb_center, target.aabb
        held.set_position_orientation(position=th.tensor(
            [float(c[0]), float(c[1]), float(thi[2]) + float(pos[2]) - float(lo[2]) + 0.01], dtype=th.float32))

    def skill(self, name: str, args: list[str]) -> dict:
        self._frames = []
        self._n = 0
        name = name.replace("-", "_").lower()
        if name not in SKILLS:
            raise SkillError(f"unknown skill {name!r}; skills: {', '.join(SKILLS)}")
        if name == "release":
            if self._held() is None:
                raise SkillError("the hand is empty")
            self._drop()
            self._settle(15, record=True)
            return {"message": "released"}
        if len(args) != 1:
            raise SkillError(f"{name} takes one object name")
        obj = self._obj(args[0])
        from omnigibson import object_states as S

        if name == "navigate_to":
            self.navigate_to(obj)
            return {"message": f"now next to {args[0]} ({self._box_dist_xy(obj):.2f} m)"}
        self._check_reach(obj)
        if name == "grasp":
            if getattr(obj, "fixed_base", False) or getattr(obj, "kinematic_only", False):
                raise SkillError(f"{args[0]} is fixed in place and cannot be picked up")
            ext = [float(v) for v in obj.aabb_extent]
            if max(ext) > MAX_GRASP_EXTENT_M:
                raise SkillError(f"{args[0]} is too big to pick up ({max(ext):.2f} m)")
            c = self._closed_container_of(obj)
            if c:
                raise SkillError(f"{args[0]} is inside {c}, which is closed; open it first")
            if self._held() is obj:
                return {"message": f"already holding {args[0]}"}
            if self._held() is not None:
                raise SkillError(f"the hand already holds {self._name(self._held())}; place or release it first")
            self._pick(obj)
            return {"message": f"grasp {args[0]}: ok"}
        elif name in ("place_on_top", "place_inside"):
            if self._held() is None:
                raise SkillError("the hand is empty; grasp something first")
            if self._held() is obj:
                raise SkillError("cannot place an object on or in itself")
            if name == "place_inside" and S.Open in obj.states and not obj.states[S.Open].get_value():
                raise SkillError(f"{args[0]} is closed; open it first")
            return self._place(obj, args[0], name)
        else:
            if self._held() is not None:
                raise SkillError(f"cannot {name} while holding {self._name(self._held())}; place or release it first")
            prim = {"open": self.P.OPEN, "close": self.P.CLOSE, "toggle_on": self.P.TOGGLE_ON,
                    "toggle_off": self.P.TOGGLE_OFF}[name]
        from omnigibson.action_primitives.action_primitive_set_base import ActionPrimitiveErrorGroup, ActionPrimitiveError

        try:
            self._run(self.prims.apply_ref(prim, obj, attempts=3))
        except (ActionPrimitiveErrorGroup, ActionPrimitiveError) as e:
            raise SkillError(f"{name} {args[0]} failed: {str(e)[:300]}")
        return {"message": f"{name} {args[0]}: ok"}

    # ---- observation -----------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        from omnigibson import object_states as S

        x, y, yaw = self._robot_xy_yaw()
        held = self._held()
        scope = self._scope()
        movable = {n: e for n, e in scope.items() if not getattr(e, "fixed_base", False)}
        out = {"robot": {"xy": [round(x, 2), round(y, 2)], "yaw_deg": round(math.degrees(yaw)),
                         "room": self._room_of_point([x, y]), "holding": self._name(held) if held is not None else None}}
        objs = {}
        for n, e in scope.items():
            d = {"category": getattr(e, "category", ""), "room": self._room(e),
                 "position": [round(float(v), 2) for v in e.aabb_center],
                 "distance_m": round(self._box_dist_xy(e), 2)}
            d["in_reach"] = d["distance_m"] <= REACH_M
            if S.Open in e.states:
                d["open"] = bool(e.states[S.Open].get_value())
            if S.ToggledOn in e.states:
                d["toggled_on"] = bool(e.states[S.ToggledOn].get_value())
            if n in movable and e is not held:
                rel = []
                for m, c in scope.items():
                    if c is e:
                        continue
                    try:
                        if S.Inside in e.states and e.states[S.Inside].get_value(c):
                            rel.append(f"inside {m}")
                        elif S.OnTop in e.states and e.states[S.OnTop].get_value(c):
                            rel.append(f"on_top_of {m}")
                    except Exception:
                        continue
                if rel:
                    d["relations"] = rel
            objs[n] = d
        out["objects"] = objs
        # names only: what a vision-only task shows (obs_mode vision, visible_fields robot + object_names), so the
        # agent can address objects in skills while their poses, states and relations must be read from the cameras
        out["object_names"] = list(scope)
        return out

    def success(self) -> dict:
        tc = self.env.task._termination_conditions["predicate"]
        done, status = tc._check_goal_fn(0)
        return {"success": bool(done), "status": {k: [int(i) for i in v] for k, v in status.items()}}

    # ---- images ----------------------------------------------------------------------------------------------------
    def _follow_rgb(self):
        """Third-person camera 2.6 m behind and 2.0 m above the robot, looking at it."""
        import torch as th
        from omnigibson.utils.transform_utils import euler2quat

        x, y, yaw = self._robot_xy_yaw()
        cx, cy = x - 2.6 * math.cos(yaw), y - 2.6 * math.sin(yaw)
        pitch = math.atan2(2.0 - 0.8, 2.6)
        # camera frame: -z forward, y up; rotate so that it looks along the robot's heading, tilted down
        q = euler2quat(th.tensor([math.pi / 2 - pitch, 0.0, yaw - math.pi / 2], dtype=th.float32))
        self.cam.set_position_orientation(th.tensor([cx, cy, 2.0], dtype=th.float32), q)
        for _ in range(2):
            self.og.sim.render()
        obs = self.cam.get_obs()[0]["rgb"]
        return obs[..., :3].cpu().numpy()

    def _head_rgb(self):
        for _ in range(2):
            self.og.sim.render()
        for name, s in self.robot.sensors.items():
            if "zed" in name or "head" in name.lower():
                return s.get_obs()[0]["rgb"][..., :3].cpu().numpy()
        s = next(iter(self.robot.sensors.values()))
        return s.get_obs()[0]["rgb"][..., :3].cpu().numpy()

    @staticmethod
    def _jpeg(img) -> str:
        from PIL import Image

        b = io.BytesIO()
        Image.fromarray(img.astype("uint8")).save(b, format="JPEG", quality=85)
        return base64.b64encode(b.getvalue()).decode()

    def render(self, camera: str = "follow") -> str:
        return self._jpeg(self._head_rgb() if camera == "head" else self._follow_rgb())

    # ---- rpc -------------------------------------------------------------------------------------------------------
    def call(self, method: str, a: dict):
        if method == "info":
            return {"activity": self.activity, "instance": self.instance, "load_s": self.load_s, "objects": self.scope_names,
                    "skills": SKILLS, "robot": self.robot.model if hasattr(self.robot, "model") else type(self.robot).__name__}
        if method == "reset":
            self._drop()
            self.env.reset()
            self._settle(30)
            return {"ok": True}
        if method == "observe":
            return self.observe()
        if method == "skill":
            try:
                r = self.skill(a.get("name", ""), [str(x) for x in a.get("args", [])])
                r["ok"] = True
            except SkillError as e:
                r = {"ok": False, "message": str(e)}
            r["frames"] = self._frames if a.get("frames", True) else []
            self._frames = []
            return r
        if method == "wait":
            self._settle(int(a.get("steps", 1)))
            return {"ok": True}
        if method == "render":
            return {"jpeg": self.render(a.get("camera", "follow"))}
        if method == "success":
            return self.success()
        if method == "debug_exec" and os.environ.get("ROBOUSE_BEHAVIOR_DEBUG") == "1":  # development only
            g = {"w": self, "og": self.og, "result": None}
            exec(a.get("code", ""), g)
            return repr(g.get("result"))[:20000]
        raise KeyError(f"unknown method {method!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--activity", required=True)
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--rooms", default="", help="comma-separated room instances to load (default: all)")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    secret = os.environ.get("ROBOUSE_BEHAVIOR_SECRET", "")
    w = Worker(a.activity, a.instance, a.scene, [r for r in a.rooms.split(",") if r] or None)
    print(json.dumps({"ready": True, "activity": a.activity, "load_s": w.load_s, "objects": w.scope_names}), flush=True)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            body = json.dumps({"ok": True, "activity": a.activity}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if secret and self.headers.get("X-Robouse-Secret") != secret:
                self.send_response(403)
                self.end_headers()
                return
            n = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
                resp = {"ok": True, "result": w.call(req.get("method", ""), req.get("args") or {})}
            except Exception as e:
                traceback.print_exc()
                resp = {"ok": False, "error": f"{type(e).__name__}: {e}"[:800]}
            body = json.dumps(resp).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    HTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
