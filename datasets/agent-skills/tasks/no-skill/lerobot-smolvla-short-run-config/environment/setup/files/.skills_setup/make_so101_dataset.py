"""Writes a tiny synthetic SO-101 LeRobotDataset (v3.0 format) to disk, without the Hub.

    python make_so101_dataset.py ROOT REPO_ID LENGTHS [--fps 30] [--no-camera]

LENGTHS is a comma-separated list of episode lengths in frames. The trajectories are smooth
interpolations between random joint targets (fixed seed); the camera is a small synthetic video.
"""
import argparse
import shutil
from pathlib import Path

import numpy as np

from lerobot.datasets import LeRobotDataset

ap = argparse.ArgumentParser()
ap.add_argument("root")
ap.add_argument("repo_id")
ap.add_argument("lengths")
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--no-camera", action="store_true")
ap.add_argument("--task", default="Pick up the red cube and place it in the bowl")
args = ap.parse_args()

root = Path(args.root)
if root.exists():
    shutil.rmtree(root)
joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
names = [f"{j}.pos" for j in joints]
features = {
    "observation.state": {"dtype": "float32", "shape": (6,), "names": names},
    "action": {"dtype": "float32", "shape": (6,), "names": names},
}
if not args.no_camera:
    features["observation.images.front"] = {
        "dtype": "video", "shape": (48, 64, 3), "names": ["height", "width", "channels"]}
ds = LeRobotDataset.create(args.repo_id, args.fps, features, root=root, robot_type="so101_follower",
                           use_videos=not args.no_camera)
rng = np.random.default_rng(0)
yy, xx = np.mgrid[0:48, 0:64]
for ep, n in enumerate(int(x) for x in args.lengths.split(",")):
    start = rng.uniform(-30, 30, 6).astype(np.float32)
    goal = rng.uniform(-30, 30, 6).astype(np.float32)
    for t in range(n):
        a = t / max(n - 1, 1)
        state = ((1 - a) * start + a * goal).astype(np.float32)
        frame = {"observation.state": state, "action": (state + 0.5).astype(np.float32), "task": args.task}
        if not args.no_camera:
            img = np.empty((48, 64, 3), dtype=np.uint8)
            img[..., 0] = (xx * 4 + t) % 256
            img[..., 1] = (yy * 5 + ep * 20) % 256
            img[..., 2] = 128
            frame["observation.images.front"] = img
        ds.add_frame(frame)
    ds.save_episode()
ds.finalize()
print(f"wrote {root}: {ds.meta.total_episodes} episodes, {ds.meta.total_frames} frames")
