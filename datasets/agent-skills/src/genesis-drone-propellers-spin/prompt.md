Two problems with the `propellers_spin` option of `gs.morphs.Drone` (in `genesis/options/morphs.py`):

1. Our config loader hands Genesis numpy arrays, and `gs.morphs.Drone(file="urdf/drones/cf2x.urdf", propellers_spin=np.array([-1, 1, -1, 1]))` fails validation, although a list or a tuple with the same values is fine.
2. Invalid spins are silently accepted: `propellers_spin=(1, 0, 1, -1)` or a spin tuple whose length differs from the number of propeller links (`propellers_link_name`) creates the morph without any error, and the drone then flies wrong or crashes much later.

Please fix both in Genesis itself:

- `propellers_spin` accepts any array-like of integers (list, tuple, numpy array) and keeps its current default.
- Creating the morph raises a `gs.GenesisException` when a spin is anything other than `1` (CCW) or `-1` (CW), or when the number of spins differs from the number of entries of `propellers_link_name`.
- Follow the project's conventions for typing option fields (the Options classes are strict Pydantic models, and `genesis/typing.py` holds the shared field types), and keep the option's documentation accurate.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
