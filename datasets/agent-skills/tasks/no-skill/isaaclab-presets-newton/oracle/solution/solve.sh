# Point the env config at the preset.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
python3 - "$F" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_physx.physics import PhysxCfg\n\n", "")
s = s.replace("import isaaclab_tasks.core.velocity.mdp as mdp\n", "import isaaclab_tasks.core.velocity.mdp as mdp\n\nfrom .physics_cfg import TrailWalkerPhysicsCfg\n")
s = s.replace("SimulationCfg(physics=PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15))", "SimulationCfg(physics=TrailWalkerPhysicsCfg())")
s = s.replace("The task only runs on PhysX for now.", "Select the physics backend with ``physics=physx`` (default) or ``physics=newton_mjwarp``.")
open(p, "w").write(s)
PY
