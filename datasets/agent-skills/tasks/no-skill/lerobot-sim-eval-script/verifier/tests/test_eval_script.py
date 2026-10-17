"""lerobot-sim-eval-script: run the user's script with a fake lerobot-eval and parse its arguments with
LeRobot's own CLI parser (EvalPipelineConfig; nothing is simulated)."""
import pytest

from lr_helpers import W, capture_cli, parse_eval

SCRIPT = W / "scripts_local" / "eval_act_aloha.sh"
CKPT = W / "checkpoints_from_gpu_box" / "act_aloha_transfer" / "checkpoints" / "100000" / "pretrained_model"


@pytest.fixture(scope="module")
def argv():
    assert SCRIPT.is_file(), f"missing {SCRIPT}"
    r, calls = capture_cli(SCRIPT, commands=("lerobot-eval", "lerobot-train", "lerobot-record", "lerobot-rollout"))
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert [c["cmd"] for c in calls] == ["lerobot-eval"], f"expected exactly one lerobot-eval call, got {calls}"
    return calls[0]["argv"]


@pytest.fixture(scope="module")
def cfg(argv):
    return parse_eval(argv)


def _arg(argv, name):
    val = None
    for i, a in enumerate(argv):
        if a.startswith(f"--{name}="):
            val = a.split("=", 1)[1]
        elif a == f"--{name}" and i + 1 < len(argv):
            val = argv[i + 1]
    return val


def test_loads_the_local_checkpoint(cfg):
    assert cfg.policy is not None and cfg.policy.type == "act"
    assert cfg.policy.pretrained_path is not None
    assert (W / cfg.policy.pretrained_path).resolve() == CKPT.resolve()


def test_matching_sim_env_and_task(cfg):
    assert cfg.env.type == "aloha"
    assert cfg.env.task == "AlohaTransferCube-v0"


def test_enough_episodes(cfg):
    assert cfg.eval.n_episodes >= 50


def test_parallel_envs(cfg):
    assert cfg.eval.batch_size == 10


def test_gpu(argv):
    device = _arg(argv, "policy.device")
    assert device is not None and device.startswith("cuda"), device


def test_output_dir(cfg):
    assert (W / cfg.output_dir).resolve() == (W / "outputs" / "eval" / "act_aloha_transfer").resolve()
