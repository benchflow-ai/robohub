I wrote a tiny gymnasium environment for smoke-testing policies without a simulator: `Reach1DEnv` in `src/lerobot/envs/reach1d.py` (a point on a line has to reach a target; read the file). Please hook it into LeRobot as the env type `reach1d`, so that `lerobot-eval --env.type=reach1d ...` and `lerobot-train --env.type=reach1d ...` can use it.

What I need:

- `reach1d` must be available as soon as `lerobot.envs` is imported (the CLIs do that), configurable from the command line with these fields and defaults: `task="Reach1D-v0"`, `fps=10`, `episode_length=50` (maximum steps per episode), `tolerance=0.05` (passed to the env).
- Features: the action is a 1-D `action` feature of shape `(1,)`, and the env's `agent_pos` (shape `(2,)`) is the robot state, mapped to `observation.state`.
- `lerobot.envs.make_env(cfg, n_envs=N, use_async_envs=...)` returns `{"reach1d": {0: vec_env}}` with `N` copies of the env, an async vector env when asked for one with `N > 1`.
- `lerobot-eval`'s rollout loop must run on it unchanged, stop each episode after `episode_length` steps at most, and count an episode as a success when the point reaches the target and as a failure when it runs out of steps (the env currently reports nothing about success).

Keep `Reach1DEnv`'s dynamics as they are.
