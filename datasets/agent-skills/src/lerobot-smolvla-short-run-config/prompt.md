I recorded a small pick-and-place dataset with my SO-101. It is on disk at `recordings/so101_pick_cube` (repo id `mira/so101_pick_cube`; it is not on the Hub). I want to train SmolVLA on it on our workstation (one 24 GB GPU), and I want the run described in a config file instead of a long command line.

Please write `train_configs/smolvla_pick_cube.yaml` so that `lerobot-train --config_path=train_configs/smolvla_pick_cube.yaml`, run from the repository root with no other flags, does this:

- Policy: a fresh SmolVLA built from the `smolvla` policy type (no `path` to a pretrained checkpoint; the training box is offline, so parsing the config must not download anything).
- Exactly 6 epochs over the whole dataset at batch size 8. One step is one batch; a partial last batch counts as a step. Work the step count out from the dataset itself.
- A learning-rate schedule sized for this run: warmup over the first 5% of the training steps (rounded down), and the cosine decay finishing exactly at the last step.
- A checkpoint every 2 epochs (plus the final one).
- The vision encoder must actually be trained, not frozen. I have read that this helps a lot on specialised tasks.
- Nothing is pushed to the Hub.
- Outputs go to `outputs/train/smolvla_pick_cube`, with job name `smolvla_pick_cube`.
