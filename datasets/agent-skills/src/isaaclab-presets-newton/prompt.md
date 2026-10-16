Our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) only runs on PhysX. Make the physics backend selectable from the command line the way Isaac Lab's maintained tasks do, so that `isaaclab train --task IsaacContrib-TrailWalker-Flat physics=physx` and `... physics=newton_mjwarp` (Newton with the MuJoCo-Warp solver) both work.

Running without a `physics=` selector must keep using PhysX exactly as today, including our `gpu_max_rigid_patch_count=10 * 2**15` setting. Do not change the rest of the task.
