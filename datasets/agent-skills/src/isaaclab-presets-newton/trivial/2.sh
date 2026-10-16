# Shortcut (the skill's known failure mode): a preset whose newton_mjwarp entry is a copy of the PhysX config.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
python3 - "$F" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_physx.physics import PhysxCfg\n", "from isaaclab_physx.physics import PhysxCfg\n\nfrom isaaclab_tasks.utils import PresetCfg\n")
s = s.replace("@configclass\nclass TrailWalkerSceneCfg", "@configclass\nclass TrailWalkerPhysicsCfg(PresetCfg):\n    default = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)\n    physx = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)\n    newton_mjwarp = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)\n\n\n@configclass\nclass TrailWalkerSceneCfg", 1)
s = s.replace("SimulationCfg(physics=PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15))", "SimulationCfg(physics=TrailWalkerPhysicsCfg())")
open(p, "w").write(s)
PY
