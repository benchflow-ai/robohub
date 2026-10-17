Sensor read delays are off by one step for some values. With `gs.options.SimOptions(dt=0.05)`, an IMU created with `gs.sensors.IMU(entity_idx=..., delay=0.15)` returns readings that are only 2 steps old instead of 3, and adding it to the scene logs the warning "Read delay should be a multiple of the simulation time step", although 0.15 s is exactly 3 steps of 0.05 s. `delay=0.3` has the same problem, while `delay=0.1` works.

Please fix this in Genesis so that a delay that is a multiple of the time step (up to floating-point rounding) gives exactly that number of steps of delay, for every sensor, and is accepted without a warning. A delay that really is not a multiple of the time step (say 0.125 s with `dt=0.05`) must still log that warning. Follow the project's conventions for comparing floating-point values.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
