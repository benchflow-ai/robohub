Our Pole Balance task (`source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/`, Gym ID `IsaacContrib-PoleBalance-Direct`) is a direct-workflow task built on Isaac Lab's direct cartpole environment. Add domain randomization to it, without changing the cartpole environment class:

- once when the simulation starts, scale the mass of the pole body by a random factor drawn uniformly from [0.8, 1.2] (and keep its inertia consistent with the new mass);
- once when the simulation starts, set the friction of the cart-to-pole joint to a random value drawn uniformly from [0.0, 0.05].

Use Isaac Lab's built-in randomization terms. Only the pole's mass and that one joint's friction should change.
