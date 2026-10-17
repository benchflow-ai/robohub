My laptop has no GPU at all (CPU only), but I want to train ACT on my SO-101 dataset, which is already on the Hub as `mira/so101_sort_socks`. My Hugging Face username is `mira` and I am logged in with `hf auth login`.

Please write `scripts_local/train_sort_socks.sh`, a bash script run from the repository root that starts this training with `lerobot-train` (call `lerobot-train` itself, once, not through `uv run` or `python -m`):

- It must not train on this laptop. Run it on Hugging Face's own managed GPU hardware, on the smallest flavor with a single 24 GB NVIDIA A10G.
- Batch size 8 and 20000 steps.
- Stop the remote run after at most 3 hours, whatever happens.
- I want to close my laptop after starting it, so the command must not stay attached to the run's logs.
- The trained policy goes to the Hub repo `mira/act_sort_socks`.

The script must not need any environment variables or arguments.
