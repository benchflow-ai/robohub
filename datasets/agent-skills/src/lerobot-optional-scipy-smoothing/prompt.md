I replay recorded SO-101 action trajectories and they are jittery, so I'd like a smoothing helper in LeRobot itself. Please add `src/lerobot/utils/action_smoothing.py` with:

`smooth_actions(actions, window_length: int = 9, polyorder: int = 2)`

- `actions` is a NumPy array or a torch tensor of shape `(T, D)` (time first). It returns the same kind of object (a tensor keeps its dtype and device) with each dimension smoothed over time by a Savitzky-Golay filter, computed with `scipy.signal.savgol_filter(..., window_length, polyorder, axis=0, mode="interp")`.

scipy is not a core LeRobot dependency: it comes with the optional `scipy-dep` extra, and it is not installed in this environment. So:

- `import lerobot.utils.action_smoothing` must work when scipy is not installed.
- Calling `smooth_actions` without scipy must raise an `ImportError` that tells the user to install the `scipy-dep` extra of lerobot.
- Follow the conventions LeRobot already uses for optional dependencies in its own modules.
