"""Reach1D: a tiny gymnasium env for smoke-testing policies without a simulator.

A point moves on a line. The observation is [position, target]; the action is a velocity in [-1, 1]
that moves the point by `step_size * action` per step. The episode terminates once the point is
within `tolerance` of the target.
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class Reach1DEnv(gym.Env):
    metadata = {"render_modes": [], "render_fps": 10}

    def __init__(self, tolerance: float = 0.05, step_size: float = 0.1):
        super().__init__()
        self.tolerance = tolerance
        self.step_size = step_size
        self.observation_space = spaces.Dict(
            {"agent_pos": spaces.Box(low=-2.0, high=2.0, shape=(2,), dtype=np.float32)}
        )
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self._pos = 0.0
        self._target = 0.0

    def _obs(self):
        return {"agent_pos": np.array([self._pos, self._target], dtype=np.float32)}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._pos = float(self.np_random.uniform(-1.0, 1.0))
        # Always start at least 0.3 away from the target.
        offset = float(self.np_random.uniform(0.3, 0.8)) * (1 if self.np_random.random() < 0.5 else -1)
        self._target = float(np.clip(self._pos + offset, -1.5, 1.5))
        if abs(self._target - self._pos) < 0.3:
            self._target = self._pos - offset
        return self._obs(), {}

    def step(self, action):
        a = float(np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[0], -1.0, 1.0))
        self._pos += self.step_size * a
        dist = abs(self._pos - self._target)
        terminated = dist < self.tolerance
        reward = -dist
        return self._obs(), reward, terminated, False, {"distance": dist}
