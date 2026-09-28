"""Procedural desk, mount plates and bowls shared by every scene in this package."""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

from .layout import BASE_X, BOWL_X, BOWL_Y, REAR_BASE_Y


@dataclass
class Scene:
    spec: object
    arms: dict
    task_id: str
    embodiment: str
    seed: int
    objects: list = field(default_factory=list)
    goal: dict = field(default_factory=dict)
    physics: dict = field(default_factory=dict)


def geom(parent, **attrs):
    return ET.SubElement(parent, "geom", {k: str(v) for k, v in attrs.items()})


def new_root(model_name):
    root = ET.Element("mujoco", model=model_name)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    option = ET.SubElement(root, "option", timestep=".002", integrator="implicitfast",
                           cone="elliptic", impratio="10")
    # Flat cylinder bases resting on a box need multiple contact points; single
    # contact convex collision detection produces false rocking at rest.
    ET.SubElement(option, "flag", multiccd="enable")
    assets = ET.SubElement(root, "asset")
    ET.SubElement(assets, "material", name="workspace_matte", specular="0", shininess="0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "headlight", ambient=".45 .45 .45", diffuse=".2 .2 .2", specular="0 0 0")
    defaults = ET.SubElement(root, "default")
    geom(defaults, friction="1 .005 .0005", solref=".01 1", condim="6", material="workspace_matte")
    return root


def add_desk(root, *, bowls=("left", "right")):
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -.4 2", ambient=".05 .05 .05",
                  diffuse=".3 .3 .3", specular="0 0 0")
    ET.SubElement(world, "camera", name="overview", pos="1.30 -1.85 1.45",
                  xyaxes=".818 .575 0 -.29 .41 .86")
    ET.SubElement(world, "camera", name="overhead", pos="0 -.05 1.8",
                  xyaxes="1 0 0 0 1 0", fovy="48")
    # robouse addition: a closer front view of the working area for agents and videos
    ET.SubElement(world, "camera", name="workspace", pos="0 -.82 .64",
                  xyaxes="1 0 0 0 .73 .9", fovy="52")
    # robouse addition: a top-down view of the whole working area (+y, the robots' side, at the top of the image)
    ET.SubElement(world, "camera", name="top", pos="0 -.08 1.05", xyaxes="1 0 0 0 1 0", fovy="45")
    geom(world, name="table", type="box", pos="0 0 -.025", size=".8 .47 .025",
         rgba=".31 .25 .18 1")
    geom(world, name="floor", type="plane", pos="0 0 -.75", size="3 3 .1",
         rgba=".18 .20 .23 1")
    for x in (-.70, .70):
        for y in (-.37, .37):
            geom(world, name=f"desk_leg_{x}_{y}", type="box", pos=f"{x} {y} -.40",
                 size=".025 .025 .35", rgba=".12 .14 .16 1")
    for side, x in BASE_X.items():
        geom(world, name=f"mount_plate_{side}", type="box", pos=f"{x} {REAR_BASE_Y} .0075",
             size=".11 .10 .0075", rgba=".30 .33 .37 1")
    for side in bowls:
        add_bowl(world, side)
    return world


def add_bowl(world, side):
    name = f"bowl_{side}"
    body = ET.SubElement(world, "body", name=name, pos=f"{BOWL_X[side]} {BOWL_Y} 0")
    geom(body, name=name + "_floor", type="cylinder", size=".14 .004", pos="0 0 .004",
         rgba=".2 .40 .52 1")
    # A segmented concave boundary, not a convex mesh that would fill the bowl.
    for i in range(32):
        a = 2 * math.pi * i / 32
        geom(body, name=f"{name}_wall_{i}", type="box", size=".005 .014 .013",
             pos=f"{.14 * math.cos(a)} {.14 * math.sin(a)} .017",
             euler=f"0 0 {a}", rgba=".23 .48 .6 1")
    return body


def desk_geom_names(root):
    """Names of the static desk surfaces a gripper is not supposed to press."""
    world = root.find("worldbody")
    names = []
    for element in world.iter("geom"):
        name = element.get("name", "")
        if name.startswith(("table", "mount_plate", "desk_leg", "board", "bowl_")):
            names.append(name)
    return names


def object_rng(seed, conditions):
    """Stream used for the per-episode physical condition of the objects."""
    return np.random.default_rng(int(conditions.get("object_rng_seed", int(seed) ^ 0x9E3779B9)))


def yaw_quat(angle):
    return (math.cos(angle / 2), 0., 0., math.sin(angle / 2))


def jitter_pose(rng, position, *, radius, yaw):
    angle = rng.uniform(-math.pi, math.pi)
    distance = radius * math.sqrt(rng.uniform(0., 1.))
    offset = np.array([distance * math.cos(angle), distance * math.sin(angle), 0.])
    return tuple(np.asarray(position, dtype=float) + offset), yaw_quat(rng.uniform(-yaw, yaw))
