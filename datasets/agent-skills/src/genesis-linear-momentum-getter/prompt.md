For a momentum-based balance controller I need the linear momentum of rigid entities while the simulation runs, for all parallel environments at once.

Please add it to Genesis:

- `RigidSolver.get_linear_momentum(links_idx=None, envs_idx=None)` in `genesis/engine/solvers/rigid/rigid_solver.py`: the total linear momentum, in kg*m/s and in the world frame, of the given links (global link indices, all links if None), i.e. the sum over the links of their mass times the velocity of their center of mass. It returns a torch tensor of shape `(3,)` for a scene without parallel environments and `(n_envs, 3)` otherwise (`(len(envs_idx), 3)` when `envs_idx` is given), and it must be computed for all links and environments at once, like the energy getters of the solver.
- `RigidEntity.get_linear_momentum(envs_idx=None)` in `genesis/engine/entities/rigid_entity/rigid_entity.py`: the same quantity for all the links of the entity, delegating to the solver getter.

Both need docstrings. Follow the project's conventions for reading solver data in getters.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
