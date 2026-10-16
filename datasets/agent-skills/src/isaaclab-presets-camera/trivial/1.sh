# Shortcut (known failure mode): a depth camera preset, but the observation shape is left at 3 channels.
python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/pole_cam/pole_cam_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleSceneCfg\n", "from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleSceneCfg\nfrom isaaclab_tasks.utils import PresetCfg\n")
s = s.replace("@configclass\nclass PoleCamSceneCfg", "@configclass\nclass PoleCamCameraCfg(PresetCfg):\n    default = POLE_CAM_CFG\n    rgb = default\n    depth = POLE_CAM_CFG.replace(data_types=[\"depth\"])\n\n\n@configclass\nclass PoleCamSceneCfg")
s = s.replace("tiled_camera: CameraCfg = POLE_CAM_CFG", "tiled_camera: CameraCfg = PoleCamCameraCfg()")
open(p, "w").write(s)
PY
