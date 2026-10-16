# Shortcut: switch the task to Newton outright instead of adding a selectable preset.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
sed -i 's/^from isaaclab_physx.physics import PhysxCfg/from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg/; s/SimulationCfg(physics=PhysxCfg(gpu_max_rigid_patch_count=10 \* 2\*\*15))/SimulationCfg(physics=NewtonCfg(solver_cfg=MJWarpSolverCfg()))/' "$F"
