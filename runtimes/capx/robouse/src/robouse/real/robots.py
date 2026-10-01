"""Real-arm profiles: joints, limits, kinematics, default safety envelope and poses, one per supported robot.

metal   MakerMods Metal (6 Damiao joints + gripper, CAN, LeRobot `metal_follower` from makermods-robotics/lerobot branch
        arm/makermods-metal). Degrees, absolute. Kinematics: vendor canonical URDF (makermods-robotics/metal-python-ros
        @ ef4181f), identity joint mapping, checked against 842 recorded hardware poses (Link6 within 0.12 mm).
so101   SO-101 follower (6 Feetech STS3215, LeRobot `so_follower`; SO-100 uses the same profile with its own
        calibration). Degrees (LeRobot DEGREES mode); gripper 0-100 (LeRobot RANGE_0_100). Kinematics:
        TheRobotStudio/SO-ARM100 so101_new_calib.urdf (Apache-2.0), which matches LeRobot's calibration convention.
piper   AgileX Piper (6 joints + parallel gripper, CAN, `piper_sdk`). Degrees; gripper opening in mm (0-70).
        Joint limits from the Piper user manual; no kinematic chain shipped yet, so the workspace check uses joint
        limits only. UNTESTED on hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .kinematics import Chain, load_chain_file

D2R = math.pi / 180


@dataclass
class EnvelopeDefaults:
    """Workspace envelope in the robot base frame (metres): table floor, box, keep-out behind the base, slow zone."""

    table_z: float = 0.0  # table surface height in the base frame
    tool_clearance: float = 0.004  # the tool point may come this close to the table
    body_min_z: float = 0.05  # other monitored points stay this far above the table
    x_min: float = 0.05
    x_max: float = 0.62
    y_abs_max: float = 0.35
    z_max: float = 0.55
    behind_min_z: float = 0.09  # any monitored point with x < x_min must stay at least this high
    slow_zone: float = 0.076  # within this height above the table, speeds drop to the slow values
    joint_speed_deg_s: float = 15.0
    slow_joint_speed_deg_s: float = 6.0
    gripper_speed: float = 90.0  # gripper units per second
    windup_deg: float = 4.0  # a joint command may lead the measured position by at most this (bounds torque)
    windup_gripper: float = 8.0
    temp_warn_c: float = 55.0
    temp_stop_c: float = 65.0
    watchdog_s: float = 0.5  # a control step slower than this is logged as a loop stall


@dataclass
class Profile:
    name: str
    display: str
    joints: list[str]
    low: list[float]
    high: list[float]
    units: list[str]
    gripper: int | list[int] | None  # index of the gripper in the joint vector (a list for bimanual rigs)
    gripper_open: float = 0.0  # value that opens the gripper fully
    gripper_closed: float = 0.0
    control_hz: float = 10.0
    home: list[float] = field(default_factory=list)  # a safe pose above the table (reach-ready)
    park: list[float] = field(default_factory=list)  # rest pose
    chain_file: str | None = None
    joint_map: dict[int, tuple[str, float, float]] = field(default_factory=dict)
    tool: tuple[str, list[float]] = ("", [0.0, 0.0, 0.0])
    points: dict[str, tuple[str, list[float]]] = field(default_factory=dict)
    ik_seeds: list[list[float]] = field(default_factory=list)
    envelope: EnvelopeDefaults = field(default_factory=EnvelopeDefaults)
    notes: str = ""
    tested_on_hardware: bool = False

    @property
    def gidx(self) -> list[int]:
        """Indices of every gripper joint."""
        if self.gripper is None:
            return []
        return list(self.gripper) if isinstance(self.gripper, (list, tuple)) else [int(self.gripper)]

    @property
    def arm_idx(self) -> list[int]:
        """Indices of the non-gripper joints."""
        return [i for i in range(len(self.joints)) if i not in self.gidx]

    @property
    def arms(self) -> list[str]:
        """Arm prefixes for bimanual rigs ("left_", "right_"), or [""] for one arm."""
        pre = sorted({n.split("_", 1)[0] + "_" for n in self.joints if n.startswith(("left_", "right_"))})
        return pre or [""]

    def chain(self) -> Chain | None:
        if not self.chain_file:
            return None
        return Chain(load_chain_file(self.chain_file), self.joint_map, self.tool, self.points)


# MakerMods Metal: vendor soft limits (LeRobot MetalFollowerConfigBase.joint_limits) shaved by 1 deg; gripper capped at
# 115 deg (widest documented jaw opening), 0 = closed. Tool point: 0.122 m along Link6's x axis (fingertip measured on
# the bench on 2026-09-05; table at z = -0.0088 m in the base frame from the stall touchdown calibration).
METAL = Profile(
    name="metal",
    display="MakerMods Metal arm",
    joints=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"],
    low=[-159.0, -179.0, 1.0, -122.0, -84.0, -144.0, 0.0],
    high=[159.0, -1.0, 179.0, 80.0, 84.0, 144.0, 115.0],
    units=["deg"] * 7,
    gripper=6,
    gripper_open=60.0,
    gripper_closed=4.0,
    control_hz=10.0,
    home=[0.0, -110.0, 84.0, -60.0, 0.0, 0.0, 4.0],
    park=[0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 4.0],
    chain_file="metal_chain.json",
    joint_map={i: (f"joint{i + 1}", D2R, 0.0) for i in range(6)},
    tool=("link6", [0.122, 0.0, 0.0]),
    points={
        "elbow": ("link3", [0.0, 0.0, 0.0]),
        "wrist": ("link4", [0.0, 0.0, 0.0]),
        "link6": ("link6", [0.0, 0.0, 0.0]),
        "grip_mid": ("link6", [0.06, 0.0, 0.0]),
    },
    ik_seeds=[
        [0.0, -120.0, 40.0, 70.0, 0.0, 0.0],
        [0.0, -130.0, 60.0, 50.0, 0.0, 0.0],
        [0.0, -100.0, 90.0, 20.0, 0.0, 0.0],
        [0.0, -150.0, 90.0, 60.0, 0.0, 0.0],
        [-40.0, -133.0, 89.0, -48.0, 0.0, 0.0],
    ],
    envelope=EnvelopeDefaults(table_z=-0.0088),
    notes=(
        "Joint zero is the vendor's parked pose: upper arm horizontal pointing backward, forearm folded over it. "
        "shoulder_lift 0 = horizontal backward, -90 = vertical, -180 = horizontal forward; elbow_flex 0 = folded, "
        "180 = straight. Gripper 0 = closed. Base frame: +x forward, +y left, +z up, metres."
    ),
    tested_on_hardware=True,
)

# SO-101 follower. LeRobot DEGREES mode puts 0 at the middle of each calibrated range; the so101_new_calib URDF uses
# the same zero. Gripper in LeRobot's RANGE_0_100 (0 = closed). Limits: URDF limits shaved by 2 deg.
SO101 = Profile(
    name="so101",
    display="SO-101 follower arm",
    joints=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
    low=[-108.0, -98.0, -94.8, -93.0, -155.2, 0.0],
    high=[108.0, 98.0, 94.8, 93.0, 160.8, 100.0],
    units=["deg"] * 5 + ["0-100"],
    gripper=5,
    gripper_open=60.0,
    gripper_closed=2.0,
    control_hz=10.0,
    home=[0.0, -60.0, 60.0, 60.0, 0.0, 2.0],
    park=[0.0, -98.0, 94.0, 70.0, 0.0, 2.0],
    chain_file="so101_chain.json",
    joint_map={
        0: ("shoulder_pan", D2R, 0.0),
        1: ("shoulder_lift", D2R, 0.0),
        2: ("elbow_flex", D2R, 0.0),
        3: ("wrist_flex", D2R, 0.0),
        4: ("wrist_roll", D2R, 0.0),
    },
    tool=("gripper_frame_link", [0.0, 0.0, 0.0]),
    points={"elbow": ("lower_arm_link", [0.0, 0.0, 0.0]), "wrist": ("wrist_link", [0.0, 0.0, 0.0])},
    ik_seeds=[[0.0, -30.0, 30.0, 70.0, 0.0], [0.0, 0.0, 40.0, 50.0, 0.0], [0.0, 20.0, 0.0, 70.0, 0.0]],
    envelope=EnvelopeDefaults(
        table_z=0.0,
        x_min=0.08,
        x_max=0.40,
        y_abs_max=0.30,
        z_max=0.40,
        body_min_z=0.03,
        behind_min_z=0.08,
        slow_zone=0.04,
        joint_speed_deg_s=30.0,
        slow_joint_speed_deg_s=10.0,
        gripper_speed=60.0,
        windup_deg=10.0,
        windup_gripper=15.0,
        temp_warn_c=50.0,
        temp_stop_c=60.0,
    ),
    notes=(
        "LeRobot calibration: 0 deg at the middle of each joint's calibrated range. Base frame (URDF base_link): "
        "+z up, metres. Gripper 0 = closed, 100 = fully open. Forward kinematics come from the vendor URDF and have "
        "not been re-measured on this bench."
    ),
    tested_on_hardware=False,
)

# AgileX Piper: joint limits from the Piper user manual (deg); gripper opening 0-70 mm.
PIPER = Profile(
    name="piper",
    display="AgileX Piper arm",
    joints=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
    low=[-150.0, 0.0, -170.0, -100.0, -70.0, -120.0, 0.0],
    high=[150.0, 180.0, 0.0, 100.0, 70.0, 120.0, 70.0],
    units=["deg"] * 6 + ["mm"],
    gripper=6,
    gripper_open=60.0,
    gripper_closed=0.0,
    control_hz=20.0,
    home=[0.0, 30.0, -60.0, 0.0, 30.0, 0.0, 0.0],
    park=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    envelope=EnvelopeDefaults(
        joint_speed_deg_s=20.0, slow_joint_speed_deg_s=8.0, windup_deg=6.0, windup_gripper=10.0, gripper_speed=40.0
    ),
    notes="Piper joint limits from the user manual. No kinematic chain yet: only joint limits and speed limits are enforced.",
    tested_on_hardware=False,
)


def _bimanual(p: Profile, name: str, display: str) -> Profile:
    """Two copies of a single-arm profile as one rig (LeRobot's bi_* followers): left_* then right_* joints. No
    kinematic chain across both arms yet, so the workspace check is joint limits + speed only."""
    n = len(p.joints)
    return Profile(
        name=name,
        display=display,
        joints=[f"left_{j}" for j in p.joints] + [f"right_{j}" for j in p.joints],
        low=p.low * 2,
        high=p.high * 2,
        units=p.units * 2,
        gripper=[p.gripper, p.gripper + n],
        gripper_open=p.gripper_open,
        gripper_closed=p.gripper_closed,
        control_hz=p.control_hz,
        home=p.home * 2,
        park=p.park * 2,
        envelope=p.envelope,
        notes=(
            f"Two {p.display}s facing the bench from its far edge, left and right as the operator sees them. "
            "Each arm: " + p.notes
        ),
        tested_on_hardware=False,
    )


SO101_BI = _bimanual(SO101, "so101-bimanual", "bimanual SO-101 follower arms")

PROFILES = {p.name: p for p in (METAL, SO101, PIPER, SO101_BI)}
PROFILES["so100"] = SO101


def profile(name: str) -> Profile:
    try:
        return PROFILES[name]
    except KeyError:
        raise KeyError(f"unknown robot {name!r}; known: {sorted(PROFILES)}") from None
