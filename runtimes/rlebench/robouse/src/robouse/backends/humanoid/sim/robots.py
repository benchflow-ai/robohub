"""The four humanoids of the humanoid suite (MuJoCo Menagerie, pinned commit 8161bba) and how the suite drives them.

Every robot faces +x. Its root body (pelvis or trunk) is mounted on a support stand: the free joint is removed, so the
root is fixed to the world and the robot does not balance. The legs, waist and neck are held at a standing pose by
their own actuators; the arms are driven by the operational-space controller in control.py.

Per robot:
  arms      joint names per side, shoulder to wrist (the IK chain)
  ee        body and offset of the controlled end-effector point per side (the grasp point between the fingers for
            the G1, the palm plate centre for Apollo, the fist or forearm tip for the H1 and T1)
  orient    True: the arm also holds a fixed hand orientation (7-DoF arms); False: position only (4-DoF arms)
  hand      G1 Dex3 finger joints per side with their open and closed targets (grip -1 .. +1 interpolates)
  head      [yaw, pitch] joints for look_at (None if the robot has no neck)
  motor     torque-motor robots (H1): joint PD gains used by the firmware-style servo
"""

from __future__ import annotations

ROBOTS: dict[str, dict] = {
    "g1": dict(
        folder="unitree_g1",
        xml="g1_with_hands.xml",
        root="pelvis",
        keyframe="stand",
        licence="BSD-3-Clause",
        name="Unitree G1 with Dex3-1 hands",
        height_m=1.32,
        stand={"waist_pitch_joint": 0.45},
        arms={
            s: [
                f"{s}_shoulder_pitch_joint",
                f"{s}_shoulder_roll_joint",
                f"{s}_shoulder_yaw_joint",
                f"{s}_elbow_joint",
                f"{s}_wrist_roll_joint",
                f"{s}_wrist_pitch_joint",
                f"{s}_wrist_yaw_joint",
            ]
            for s in ("left", "right")
        },
        ee={"left": ("left_wrist_yaw_link", (0.12, -0.04, 0.0)), "right": ("right_wrist_yaw_link", (0.12, 0.04, 0.0))},
        orient=True,
        touch_root={"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"},
        mount="pelvis fixed to a support stand at the origin, waist bent 26 deg forward; the robot does not balance or walk",
        hand={
            s: {
                "joints": [
                    f"{s}_hand_thumb_0_joint",
                    f"{s}_hand_thumb_1_joint",
                    f"{s}_hand_thumb_2_joint",
                    f"{s}_hand_middle_0_joint",
                    f"{s}_hand_middle_1_joint",
                    f"{s}_hand_index_0_joint",
                    f"{s}_hand_index_1_joint",
                ]
            }
            for s in ("left", "right")
        },
        head=None,
        motor=None,
    ),
    "h1": dict(
        folder="unitree_h1",
        xml="h1.xml",
        root="pelvis",
        keyframe=None,
        licence="BSD-3-Clause",
        name="Unitree H1",
        height_m=1.8,
        stand={},
        arms={
            s: [f"{s}_shoulder_pitch", f"{s}_shoulder_roll", f"{s}_shoulder_yaw", f"{s}_elbow"]
            for s in ("left", "right")
        },
        ee={"left": ("left_elbow_link", (0.28, 0.0, -0.015)), "right": ("right_elbow_link", (0.28, 0.0, -0.015))},
        orient=False,
        hand=None,
        head=None,
        touch_root={"left": "left_elbow_link", "right": "right_elbow_link"},
        mount="pelvis fixed to a support stand at the origin; the robot does not balance or walk",
        # firmware-style joint servo on the torque motors: tau = kp (q_des - q) - kd qdot + gravity, clipped to the
        # motor's torque limit (ctrlrange)
        motor=dict(
            kp={"hip": 300.0, "knee": 300.0, "ankle": 60.0, "torso": 300.0, "shoulder": 300.0, "elbow": 200.0},
            kd={"hip": 15.0, "knee": 15.0, "ankle": 4.0, "torso": 15.0, "shoulder": 8.0, "elbow": 5.0},
        ),
    ),
    "apollo": dict(
        folder="apptronik_apollo",
        xml="apptronik_apollo.xml",
        root="base_link",
        keyframe="stand",
        licence="Apache-2.0",
        name="Apptronik Apollo",
        height_m=1.73,
        stand={"torso_pitch": 0.25},
        arms={
            s: [
                f"{s}_shoulder_aa",
                f"{s}_shoulder_ie",
                f"{s}_shoulder_fe",
                f"{s}_elbow_fe",
                f"{s}_wrist_roll",
                f"{s}_wrist_yaw",
                f"{s}_wrist_pitch",
            ]
            for s in ("l", "r")
        },
        ee={
            "l": ("l_wrist_pitch_link", (0.0075, -0.025, -0.112)),
            "r": ("r_wrist_pitch_link", (0.0075, 0.025, -0.112)),
        },
        orient=True,
        hand=None,
        head=["neck_yaw", "neck_pitch"],
        motor=None,
        touch_root={"l": "l_wrist_roll_link", "r": "r_wrist_roll_link"},
        mount="pelvis fixed to a support stand at the origin, torso pitched 14 deg forward; the robot does not balance or walk",
    ),
    "t1": dict(
        folder="booster_t1",
        xml="t1.xml",
        root="Trunk",
        keyframe="home",
        licence="Apache-2.0",
        name="Booster T1",
        height_m=1.18,
        root_pitch=0.35,  # mounted leaning 20 deg forward over the table; the hips compensate so the legs stay vertical
        stand={"Left_Hip_Pitch": -0.55, "Right_Hip_Pitch": -0.55},
        arms={
            s: [f"{s}_Shoulder_Pitch", f"{s}_Shoulder_Roll", f"{s}_Elbow_Pitch", f"{s}_Elbow_Yaw"]
            for s in ("Left", "Right")
        },
        ee={"Left": ("left_hand_link", (0.0, 0.2, 0.0)), "Right": ("right_hand_link", (0.0, -0.2, 0.0))},
        orient=False,
        hand=None,
        head=["AAHead_yaw", "Head_pitch"],
        motor=None,
        touch_root={"Left": "left_hand_link", "Right": "right_hand_link"},
        mount="trunk fixed to a support stand at the origin, leaning 20 deg forward; the robot does not balance or walk",
    ),
}

# the suite's side names -> each model's own prefix
SIDE_KEY = {
    "g1": {"left": "left", "right": "right"},
    "h1": {"left": "left", "right": "right"},
    "apollo": {"left": "l", "right": "r"},
    "t1": {"left": "Left", "right": "Right"},
}
