When I turn on rolling friction with `gs.options.RigidOptions(enable_rolling_friction=True)`, Genesis raises `'enable_rolling_friction' requires 'enable_torsional_friction'.` and I have to add `enable_torsional_friction=True` by hand. In that case there is only one value that works, so I should not have to set it.

Please change Genesis (`genesis/options/solvers.py`, and anything else that needs it) so that:

- `enable_torsional_friction` is resolved automatically when the user does not set it: once a `RigidOptions` is created, `options.enable_torsional_friction` reads `True` if rolling friction is enabled and `False` otherwise.
- A value the user sets explicitly is kept, except that explicitly asking for rolling friction with `enable_torsional_friction=False` still raises a `gs.GenesisException`.
- A scene built with `gs.options.RigidOptions(enable_rolling_friction=True)` alone simulates rolling friction (a rolling sphere with a non-zero `friction_rolling` material slows down).

Follow the project's conventions for options whose value is resolved automatically, including how such an option is declared and documented in the `RigidOptions` docstring.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
