Move our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) from the flat plane to rough terrain and give the policy terrain perception:

- generate the ground with Isaac Lab's standard rough-terrain generator config (`ROUGH_TERRAINS_CFG`) instead of the plane;
- mount a height scanner on the robot's base: a grid of rays with 0.1 m resolution covering 1.6 m by 1.0 m, cast down from 20 m above the base and aligned with the base's yaw only, hitting the terrain;
- add the height scan to the policy observations, clipped to [-1, 1];
- refresh the scanner once per policy step (every `decimation` physics steps).

Keep the Gym ID as it is.
