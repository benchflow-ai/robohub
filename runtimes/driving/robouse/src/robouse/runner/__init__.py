"""The local runner (`robouse run` / `run-many`, `--engine local`): each trial runs the episode server and the agent as
processes on this machine, the agent under a macOS Seatbelt profile, and writes a BenchFlow-style trial folder.

  trial.py     one trial: workspace, episode server, harness, verifier, artifacts
  jobs.py      sets of trials: selection, seeds, epochs, retries, resume
  sandbox.py   the Seatbelt profile
  cost.py      tokens and USD per trial
  atif.py      ATIF trajectories from the harnesses' raw output
"""

from .jobs import parse_seeds, run_many, select_tasks
from .trial import run_trial

__all__ = ["parse_seeds", "run_many", "run_trial", "select_tasks"]
