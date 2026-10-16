Our Pole Cam task (`source/isaaclab_tasks/isaaclab_tasks/contrib/pole_cam/`, Gym ID `IsaacContrib-PoleCam-Direct`) trains the cartpole from 128x128 RGB images. We also want to train it from depth images with the same camera.

Expose both versions as selectable variants of this task in the standard Isaac Lab way, so that `isaaclab train --task IsaacContrib-PoleCam-Direct presets=rgb` gives the RGB version and `... presets=depth` the depth version. Without a selector the task must stay the RGB version. Make sure each variant's observation space matches its image (channels first, as now).
