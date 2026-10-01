import numpy as np
open_gripper()
grasp_pos, grasp_quat = sample_grasp_pose("milk")
grasp_quat = np.array([0.0, 1.0, 0.0, 0.0])
goto_pose(grasp_pos, grasp_quat, z_approach=0.1)
close_gripper()
goto_pose(grasp_pos+np.array([0.0, 0.0, 0.2]), grasp_quat)

basket_pos, basket_quat = get_object_pose("basket")
basket_quat = np.array([0.0, 1.0, 0.0, 0.0])
goto_pose(basket_pos, basket_quat, z_approach=0.3)
open_gripper()
