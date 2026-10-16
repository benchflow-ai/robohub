python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/heading.py <<'PY'
import sys
cfg, heading = sys.argv[1], sys.argv[2]
s = open(cfg).read()
s = s.replace("rot=(1.0, 0.0, 0.0, 0.0)", "rot=(0.0, 0.0, 0.0, 1.0)")
s = s.replace("rot=(0.7071068, 0.0, 0.0, 0.7071068)", "rot=(0.0, 0.0, 0.7071068, 0.7071068)")
open(cfg, "w").write(s)
s = open(heading).read()
s = s.replace("written for Isaac Lab 2.3", "quaternions in Isaac Lab 3.0's (x, y, z, w) order")
s = s.replace("    quat[:, 0] = torch.cos(0.5 * yaw)\n    quat[:, 3] = torch.sin(0.5 * yaw)\n", "    quat[:, 2] = torch.sin(0.5 * yaw)\n    quat[:, 3] = torch.cos(0.5 * yaw)\n")
s = s.replace("    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]\n", "    x, y, z, w = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]\n")
open(heading, "w").write(s)
PY
