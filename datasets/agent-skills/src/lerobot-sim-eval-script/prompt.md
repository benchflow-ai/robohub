I trained an ACT policy on our GPU box and copied the final checkpoint here: `checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model` (only the config files so far; the weights are still copying, so you cannot run it yet).

Please write `scripts_local/eval_act_aloha.sh`, a bash script run from the repository root that evaluates this checkpoint with `lerobot-eval` (call `lerobot-eval` itself, once, not through `uv run` or `python -m`):

- Evaluate it in the simulated environment and task it was trained for (check the checkpoint, don't guess).
- Run enough episodes that the success rate is a stable estimate I can compare with published numbers.
- 10 environments in parallel, policy on the GPU (`cuda`).
- Results in `outputs/eval/act_aloha_transfer`.

The script must not need any environment variables or arguments.
