# agent-skills: do repository agent guides help coding agents?

Simulator and robot-learning teams now ship guidance written for coding agents: Isaac Lab 3.0 has 17 user skills with written pass/fail scenarios (`skills/user/*/evaluations.md`), LeRobot has `AGENT_GUIDE.md` and `AGENTS.md`, Genesis has `CLAUDE.md`. Nobody has published whether they help. This folder turns that guidance into bench tasks and prepares the study that measures it: Claude Code and Codex, with and without the guides, 5 trials per cell. [`STUDY.md`](STUDY.md) has the plan, the exact commands, trial counts and cost estimates; [`TASKS.md`](TASKS.md) lists the tasks; [`PROVENANCE.md`](PROVENANCE.md) the sources and licences.

This is not a Robo Use dataset (the agent does not drive a robot), so it is not in `hub.yaml` or `registry.json`, and `scripts/export.py` leaves this folder alone. Run it with `bench eval run --tasks-dir`.

## What a task is

Each task gives the agent a real job in the upstream repository at a pinned commit: migrate a 2.x Isaac Lab project to 3.0, add a Newton preset, wire a contact sensor, register a task; add a LeRobot env config, processor step or training config the way the project expects; fix or extend Genesis following its conventions. A hidden pytest verifier decides pass or fail deterministically: it loads configs through the same entry points as the command line, calls the changed functions on CPU tensors, builds and trains RSL-RL runners for one iteration, steps Genesis scenes on the CPU backend, and so on. Everything runs on CPU in a `python:3.12-slim-bookworm` image with CPU-only PyTorch; no GPU, Isaac Sim or downloaded assets are needed.

Every task exists in two variants with the same task name:

- `tasks/with-skill/<id>/`: the repository exactly as shipped, guides included (for Isaac Lab, also the `.agents/skills` and `.claude/skills` aliases that Codex and Claude Code discover automatically);
- `tasks/no-skill/<id>/`: the same, with the guide files deleted before the workspace is committed as a fresh one-commit git repository (no history to recover them from). The build fails if any guide file is left.

The verifier also fails a trial that edits protected upstream files (an integrity manifest the agent cannot read), so a task cannot be passed by changing the library under test.

## Layout

- `sources.yaml`: upstream repository, pinned commit, licence, guide files and protected paths per source.
- `src/<id>/`: the task source (prompt, starting state, reference solution, hidden tests, trivial edits that must fail, and for some tasks valid alternative solutions that must pass); `src/_shared/`: a starting project and test helpers shared by several tasks.
- `env/<source>/requirements.txt`: the pinned Python environment of each source.
- `tasks/`: the built bench tasks (`python3 tools/build.py` regenerates them from `src/`; do not edit them by hand).
- `tools/check.py`: the checks (reference and valid alternatives pass, no-op and trivial edits fail, both variants); `checks/*.jsonl` and [`CHECKS.md`](CHECKS.md): their results.
- `study/`: the four job files (agent x variant) and `run_study.sh`; `tools/summarize.py`: the analysis.

## Adding a task

Write `src/<id>/` (see `tools/build.py` for the format), run `python3 tools/build.py <id>`, then run `tools/check.py` in a sandbox built like the task image and commit the task with its check record. A task is ready when its reference solution and valid alternatives pass in both variants and the no-op and every trivial edit fail in both.
