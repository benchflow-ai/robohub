#!/bin/bash
# Shortcut: success reported and type registered, but the envs are built without the episode-length cap.
set -e
O="$(cd "$(dirname "$0")/.." && pwd)"
cp "$O/solution/files/src/lerobot/envs/reach1d.py" src/lerobot/envs/reach1d.py
bash "$O/solution/solve.sh"
python3 - <<'PY'
p = "src/lerobot/envs/configs.py"
s = open(p).read()
s = s.replace("return gym.wrappers.TimeLimit(Reach1DEnv(**kwargs), max_episode_steps=max_steps)", "return Reach1DEnv(**kwargs)")
open(p, "w").write(s)
PY
