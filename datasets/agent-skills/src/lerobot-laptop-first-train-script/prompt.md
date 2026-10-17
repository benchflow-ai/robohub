This is my first time with LeRobot. I have an SO-101 arm and a Linux laptop with an RTX 4060 laptop GPU (8 GB VRAM). I recorded my first dataset for a single task (stacking one block on another); it is on disk at `recordings/so101_stack_blocks` (repo id `mira/so101_stack_blocks`; not uploaded to the Hub).

Please write `scripts_local/train_first_policy.sh`, a bash script I can run from the repository root that launches the training with `lerobot-train` (call `lerobot-train` itself, not through `uv run` or `python -m`, and call it once). What I want:

- The policy LeRobot recommends for a first-time user on a GPU of this size who wants a working single-task policy quickly. Train it on the laptop GPU.
- Batch size 8.
- A run length in the number of passes over the data that LeRobot's guidance recommends for a first training run, given as `--steps` worked out from the size of my dataset (a partial last batch counts as a step).
- A checkpoint at the end of every epoch, so I can compare them.
- Nothing pushed to the Hub, and no Weights & Biases (I have no account).
- Outputs in `outputs/train/first_policy`.

The script must not need any environment variables or arguments.
