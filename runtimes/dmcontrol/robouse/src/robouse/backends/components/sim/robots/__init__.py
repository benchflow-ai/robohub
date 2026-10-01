"""Embodiment drivers for composed tasks: the robot model, its placement in a scene, its controllers ("firmware"
under `robo act`), its declared action groups and skills, and the robot part of `robo observe`.

Every driver works in any scene that offers its placement (`mount`): surface-mounted arms (`arm`, `bimanual`) get a
base pose on a work surface, floor robots (`floor`) a start pose, drones (`airspace`) a start pad.

Skills are the same small vocabulary on every embodiment that has the capability, so reference solutions (and
agents) can be written once:

  manipulation (reach, grasp)   move_to X Y Z [ARM], grasp [ARM], release [ARM], home [ARM]
  wheeled or legged base        go_to X Y [YAW_DEG], turn YAW_DEG, look_at X Y Z
  flight                        takeoff Z, fly_to X Y Z, land, turn YAW_DEG, look_at X Y Z

A skill is a generator that yields one action per 50 ms control step; the episode server runs each as a normal
step (budget, video, success checks). A skill never moves the robot in a way `robo act` could not.
"""

from __future__ import annotations

from .arms import ARM_ARG, Aloha, ArmCfg, ArmRig, Panda
from .base import CONTROL_DT, GRASP_DOC, N_SUB, PHYS_DT, Driver, FloorRobot
from .drone import Crazyflie
from .mobile import GoogleRobot, MobileManip, Tiago
from .quadruped import Go2

DRIVERS = {c.key: c for c in (Panda, Aloha, Tiago, GoogleRobot, Go2, Crazyflie)}

__all__ = ["ARM_ARG", "CONTROL_DT", "DRIVERS", "GRASP_DOC", "N_SUB", "PHYS_DT", "Aloha", "ArmCfg", "ArmRig", "Crazyflie",
           "Driver", "FloorRobot", "Go2", "GoogleRobot", "MobileManip", "Panda", "Tiago"]
