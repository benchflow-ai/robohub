I want a tiny baseline policy in LeRobot to sanity-check new datasets: a state-only MLP behaviour-cloning policy that maps `observation.state` at time t to `action` at time t. There is a state-only test dataset on disk at `recordings/so101_reach_state` (repo id `mira/so101_reach_state`).

Please add it to the library as the policy type `mlp_bc`:

- Code in `src/lerobot/policies/mlp_bc/`, with the config class `MLPBCConfig` and the policy class `MLPBCPolicy`.
- Config fields (besides the common ones): `hidden_dim: int = 256`, `n_layers: int = 2` (number of hidden layers), `optimizer_lr: float = 1e-3`, `optimizer_weight_decay: float = 0.0`. Its optimizer preset is AdamW with these values; no LR scheduler. It uses a single observation step and predicts a single action (no action chunk). State and action are normalized with mean/std from the dataset stats.
- `--policy.type=mlp_bc` (with e.g. `--policy.hidden_dim=64`) must work in `lerobot-train` and `lerobot-eval` like any built-in policy, as soon as `lerobot.policies` is imported.
- LeRobot's own factories must work for it: `lerobot.policies.make_policy_config("mlp_bc")`, `get_policy_class("mlp_bc")`, `make_policy(cfg, ds_meta=...)` and `make_pre_post_processors(cfg, dataset_stats=...)`.
- The policy implements the usual training and inference interface: `forward(batch)` returns `(loss, info_dict)` with an MSE loss on the normalized action, `select_action(batch)` returns a `(B, action_dim)` action, `predict_action_chunk(batch)` returns `(B, 1, action_dim)`, and `reset()`. It must be trainable with an optimizer from its preset, and `save_pretrained` / `from_pretrained` must round-trip it.

The dataset's `observation.state` and `action` are 6-dimensional, but don't hard-code the sizes.
