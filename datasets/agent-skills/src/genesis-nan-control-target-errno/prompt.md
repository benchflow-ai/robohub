A bug in our policy sometimes produces NaN joint targets. When one of them reaches `entity.control_dofs_position(...)`, Genesis does not complain at all: the PD controller saturates and the joint is silently slammed at its force limit, so a whole training run goes on with garbage in that environment.

Please make a NaN position target halt the simulation, the same way the rigid solver already halts on its other NaN errors:

- `control_dofs_position` itself must stay cheap: it must not raise or synchronize with the host per call. Flag the problem per environment with the rigid solver's existing error-code mechanism, defining a new error code for it in `genesis/utils/array_class.py`.
- Within the solver's usual error-check cadence, `scene.step()` then raises a `gs.GenesisException` whose message says that a position control target is NaN (not the generic advice to decrease the time step), and `scene.rigid_solver.get_error_envs_mask()` reports exactly the environments that received a NaN target.
- `scene.reset()` clears the error as for the other error codes, and the simulation keeps working afterwards with valid targets.

It must work with parallel environments (`scene.build(n_envs=...)`) and on every backend; this machine only has the CPU one (`gs.init(backend=gs.cpu)`).

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
