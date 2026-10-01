# Vendored CaP-X code

Source: https://github.com/capgym/cap-x at commit 53e9966d7a8e2fa7494676772bccc35280f5c0ed, MIT licence (LICENSE, copyright 2026 Max Fu). Only what the privileged-API tasks run is copied, under CaP-X's own package paths so its imports resolve unchanged.

| Here (capx/...) | Changes |
|---|---|
| envs/base.py | none |
| envs/simulators/robosuite_{base,cube_lift,cubes,cubes_restack,spill_wipe,nut_assembly,two_arm_lift,handover}.py | none |
| envs/simulators/libero.py, integrations/libero/__init__.py | none (LIBERO-PRO itself is installed in the capx-libero runtime) |
| utils/camera_utils.py, utils/depth_utils.py | none |
| integrations/base_api.py | none |
| integrations/franka/control_privileged.py, nut_assembly_privileged.py, two_arm_lift_privileged.py, handover_privileged.py, libero_privileged.py | none |
| integrations/franka/spill_wipe_privileged.py | unused perception imports removed (Contact-GraspNet, OWL-ViT, SAM2, open3d, depth utilities) |
| integrations/franka/common.py | open3d imported inside the one function that uses it (the perception APIs' bounding boxes) |
| integrations/motion/pyroki_context.py, pyroki_snippets/ | none |
| integrations/motion/pyroki.py | replaced: CaP-X's version posts IK requests to its PyRoKi server; this one runs the server's solve in-process (same robot model, joint-limit margin, target link and solver calls) |
| integrations/robosuite/controllers/config/robots/*.json | none |
| the `__init__.py` files | empty (CaP-X's register every environment, including ones needing other simulators) |

Robo Use builds the environments with `privileged=True, enable_render=True`, seeds robosuite's layout sampler per task, and runs each API call step by step under the episode server (../worker.py).
