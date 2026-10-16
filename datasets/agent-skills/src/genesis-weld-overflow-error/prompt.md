`scene.rigid_solver.add_weld_constraint(link1_idx, link2_idx)` crashes with `AttributeError: 'ConstraintSolver' object has no attribute 'rigid_info'` as soon as I add more welds than `gs.options.RigidOptions(max_dynamic_constraints=...)` allows (try `max_dynamic_constraints=1` and two welds).

Please fix it in Genesis (`genesis/engine/solvers/rigid/constraint/solver.py`):

- Adding a weld beyond the capacity raises a `gs.GenesisException` whose message tells the user to increase the `max_dynamic_constraints` option. The new weld is not registered: I asked for a weld and did not get one, so a mere warning is not acceptable.
- The welds registered before stay active (`scene.rigid_solver.get_weld_constraints()` still lists them), and the simulation keeps running normally.
- The code of this module reads quadrants data with direct `.to_numpy()` calls in a few places. Bring the whole module in line with the project's data-access conventions while you are there.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
