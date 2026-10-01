import numpy as np

# get poses
handle_pos, handle_quat = get_hammer_pose() # privileged
handle_pos[2] -= 0.025  # handle_pos is slightly high sometimes
handle_pos[1] -= 0.035  # handle pos is in the middle

# pickup quat
gripper_down_quat = np.array([0, 1, 0, 0]) # handover quat
gripper_pick_quat = np.array([0, 0.707, 0.707, 0]) # down grip quat

# handover pos
arm0_pos = np.array([0.44, 0.0, 0.0]) # approx init positions
arm1_pos = np.array([1.18, 0.0, 0.0])
handover_pos = (arm0_pos + arm1_pos) / 2
handover_pos[2] = 0.10

# --- Sequence ---
# Arm0: pick up hammer at actual handle pose
open_gripper_arm0()
goto_pose_arm0(handle_pos, gripper_pick_quat, z_approach=0.15)

close_gripper_arm0()
goto_pose_arm0(handle_pos + np.array([0, 0, 0.1]), gripper_pick_quat)
goto_pose_arm0(handle_pos + np.array([0, 0, 0.2]), gripper_pick_quat)

# Arm0: move to handover (shifted toward arm1)
goto_pose_arm0(handover_pos, gripper_down_quat)

# Arm1 approach
arm1_quat = np.array([0, 0, 1, 0])
open_gripper_arm1()
goto_pose_arm1(handover_pos + np.array([0.1, 0, -0.01]), arm1_quat, z_approach=0.12) # account for hammer length
close_gripper_arm1()

# Arm0: release and retract
open_gripper_arm0()
goto_pose_arm0(handover_pos + np.array([-0.1, 0, 0.06]), gripper_down_quat)
