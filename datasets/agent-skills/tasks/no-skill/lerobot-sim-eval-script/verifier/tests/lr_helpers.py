"""Shared helpers for the LeRobot agent-skills verifiers (copied next to each task's tests)."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

W = Path(os.environ["SKILLS_STUDY_WORKDIR"])


def capture_cli(script, commands=("lerobot-train",), timeout=180):
    """Run a user's bash script with fake LeRobot CLIs first on PATH.

    The fakes only record their arguments, so nothing trains, downloads or touches hardware.
    Returns (CompletedProcess, [{"cmd": name, "argv": [...]}, ...]) in call order.
    """
    tmp = Path(tempfile.mkdtemp(prefix="skills_cli_"))
    bindir = tmp / "bin"
    bindir.mkdir()
    (tmp / "home").mkdir()
    log = tmp / "calls.jsonl"
    for c in commands:
        p = bindir / c
        p.write_text(
            f"#!{sys.executable}\nimport json, sys\n"
            f"with open({str(log)!r}, 'a') as f:\n"
            f"    f.write(json.dumps({{'cmd': {c!r}, 'argv': sys.argv[1:]}}) + '\\n')\n")
        p.chmod(0o755)
    env = dict(os.environ, PATH=f"{bindir}:{os.environ.get('PATH', '/usr/bin:/bin')}", HOME=str(tmp / "home"),
               HF_HUB_OFFLINE="1")
    r = subprocess.run(["bash", str(script)], cwd=W, env=env, capture_output=True, text=True, timeout=timeout,
                       stdin=subprocess.DEVNULL)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return r, calls


def _run_wrapped(config_cls, prog, argv, validate):
    import lerobot.envs  # noqa: F401  (registers env types, as the CLIs do)
    import lerobot.policies  # noqa: F401  (registers policy types, as the CLIs do)
    from lerobot.configs import parser

    def main(cfg):
        if validate:
            cfg.validate()
        return cfg

    main.__annotations__ = {"cfg": config_cls}
    wrapped = parser.wrap()(main)
    old_argv, old_cwd = sys.argv, os.getcwd()
    parser._config_path_args.clear()
    parser._config_yaml_overrides.clear()
    sys.argv = [prog, *argv]
    os.chdir(W)
    try:
        return wrapped()
    finally:
        sys.argv = old_argv
        os.chdir(old_cwd)


def parse_train(argv):
    """Parse lerobot-train arguments exactly as the CLI does, then run TrainPipelineConfig.validate()."""
    from lerobot.configs.train import TrainPipelineConfig

    return _run_wrapped(TrainPipelineConfig, "lerobot-train", argv, validate=True)


def parse_eval(argv):
    """Parse lerobot-eval arguments exactly as the CLI does (EvalPipelineConfig)."""
    from lerobot.configs.eval import EvalPipelineConfig

    return _run_wrapped(EvalPipelineConfig, "lerobot-eval", argv, validate=False)


def total_frames(root):
    return json.loads((Path(root) / "meta" / "info.json").read_text())["total_frames"]
