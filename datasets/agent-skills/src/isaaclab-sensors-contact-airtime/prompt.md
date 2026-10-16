Our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`, ANYmal-C) needs contact information. Please:

- add a contact sensor that covers the robot's bodies and keeps enough history for the terms below;
- reward foot air time (weight 0.5, air-time threshold 0.5 s, for the `base_velocity` command) on ANYmal-C's four feet;
- penalize contacts on the thighs (weight -1.0, force threshold 1.0 N);
- end the episode when the base touches the ground (force threshold 1.0 N), in addition to the existing terminations.

It must work with both PhysX and Newton later, so do not use backend-specific sensor classes. Reuse Isaac Lab's existing reward and termination terms where they fit.
