`gs.morphs.Box` accepts sizes that make no physical sense. `gs.morphs.Box(size=(-0.1, 0.2, 0.2))`, `gs.morphs.Box(size=(0.0, 0.2, 0.2))` and `gs.morphs.Box(lower=(0, 0, 0), upper=(0, 1, 1))` are all created without complaint, and the box then ends up with a negative or zero volume (so a negative or zero mass and inertia) once it is added to a scene.

Please fix this in Genesis itself (`genesis/options/morphs.py` and whatever else is appropriate):

- Creating a `Box` must raise a `gs.GenesisException` when any component of `size` is not strictly positive, or when `upper` is not strictly greater than `lower` along every axis.
- Valid boxes must keep working exactly as today, whether given by `pos`/`size` or by `lower`/`upper`, and `size`, `lower` and `upper` must keep accepting lists, tuples and numpy arrays.
- Follow the project's conventions for typing option fields (the Options classes are strict Pydantic models, and `genesis/typing.py` holds the shared field types).

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
