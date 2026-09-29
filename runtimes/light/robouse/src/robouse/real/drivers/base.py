"""Driver contract for real arms: the only code that talks to hardware (or to the hardware-in-the-loop mock).

A driver connects, reads the measured joint positions (plus telemetry and camera frames) and sends absolute joint
targets in the profile's units. It applies no safety policy of its own beyond what the vendor SDK does; the episode
server's `Guard` (real/safety.py) decides every command. Drivers are selected by name in the task's `real:` block
(`driver: lerobot | piper | mock`), and third-party drivers can register under the `robouse.drivers` entry point.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..robots import Profile


@dataclass
class Reading:
    q: np.ndarray                                    # measured joint positions, profile units
    extra: dict = field(default_factory=dict)        # telemetry: temp_c / temp_mos_c / temp_rotor_c, joint_torque_nm, ...
    images: dict = field(default_factory=dict)       # camera name -> HxWx3 uint8


class ArmDriver:
    profile: Profile
    mock: bool = False
    cameras: list[str] = []
    description: str = ""

    def connect(self) -> None:
        """Open the connection; after this the arm must hold its measured pose (never go limp or jump)."""
        raise NotImplementedError

    def read(self) -> Reading:
        raise NotImplementedError

    def send(self, q: np.ndarray) -> None:
        """Command absolute joint targets (already checked by the Guard)."""
        raise NotImplementedError

    def disconnect(self, release_torque: bool) -> None:
        raise NotImplementedError

    def hardware_id(self) -> str:
        return self.profile.name
