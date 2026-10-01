"""Train a MuJoCo Playground joystick locomotion policy with Brax PPO (on the GPU worker machine).

Usage: python train_playground.py ENV_NAME OUT_DIR [--steps N]
  e.g. python train_playground.py Go1JoystickFlatTerrain /root/policies/go1_flat

Writes OUT_DIR/params.pkl (the policy's normalizer + network parameters), OUT_DIR/config.json (environment config and
network sizes, so the policy can be rebuilt for inference) and OUT_DIR/metrics.jsonl (evaluation reward over training).
The PPO hyperparameters are Playground's own (mujoco_playground.config.locomotion_params.brax_ppo_config).
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import pickle
import time

os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.8")
os.environ.setdefault("MUJOCO_GL", "egl")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("env")
    ap.add_argument("out")
    ap.add_argument("--steps", type=int, default=0, help="override num_timesteps")
    a = ap.parse_args()
    import jax
    from brax.training.agents.ppo import networks as ppo_networks
    from brax.training.agents.ppo import train as ppo
    from mujoco_playground import registry, wrapper
    from mujoco_playground.config import locomotion_params

    os.makedirs(a.out, exist_ok=True)
    env_cfg = registry.get_default_config(a.env)
    env = registry.load(a.env, config=env_cfg)
    params = locomotion_params.brax_ppo_config(a.env)
    if a.steps:
        params.num_timesteps = a.steps
    tp = dict(params)
    net = tp.pop("network_factory", None)
    factory = functools.partial(ppo_networks.make_ppo_networks, **(dict(net) if net else {}))
    t0 = time.time()
    mf = open(os.path.join(a.out, "metrics.jsonl"), "w")

    def progress(step, metrics):
        row = {
            "step": int(step),
            "t": round(time.time() - t0, 1),
            "reward": float(metrics.get("eval/episode_reward", 0.0)),
            "reward_std": float(metrics.get("eval/episode_reward_std", 0.0)),
        }
        mf.write(json.dumps(row) + "\n")
        mf.flush()
        print(row, flush=True)

    train_fn = functools.partial(ppo.train, **tp, network_factory=factory, progress_fn=progress)
    make_inference_fn, prm, _ = train_fn(
        environment=env, eval_env=registry.load(a.env, config=env_cfg), wrap_env_fn=wrapper.wrap_for_brax_training
    )
    with open(os.path.join(a.out, "params.pkl"), "wb") as f:
        pickle.dump(jax.device_get(prm), f)
    json.dump(
        {
            "env": a.env,
            "env_config": env_cfg.to_dict(),
            "network": dict(net) if net else {},
            "num_timesteps": int(params.num_timesteps),
            "train_s": round(time.time() - t0, 1),
            "normalize_observations": bool(params.get("normalize_observations", True)),
        },
        open(os.path.join(a.out, "config.json"), "w"),
        indent=1,
        default=str,
    )
    print("done", a.env, round(time.time() - t0, 1), "s", flush=True)


if __name__ == "__main__":
    main()
