"""Settings from the environment: every `ROBOUSE_*` variable robouse reads is declared in `ENV`, with its meaning.

Code reads settings through the helpers here (`env`, `flag`, `path`, `cache_dir`, `sim_python`, ...), never through
`os.environ` directly, so the list below stays complete (tests/test_config.py checks it against the source).
Maintainer-only fallbacks (credential pools, endpoint files inside a checkout) live in `robouse._maintainer`.
"""

from __future__ import annotations

import os
from pathlib import Path

ENV: dict[str, str] = {
    # locations
    "ROBOUSE_CACHE_DIR": "cache for simulator virtualenvs, downloaded assets and harness homes (default ~/.cache/robouse)",
    "ROBOUSE_CONFIG_DIR": "private settings such as remote worker endpoints (default ~/.config/robouse); agents never "
    "see it",
    "ROBOUSE_MENAGERIE_ASSETS": "MuJoCo Menagerie robot models (default: the checkout's assets/menagerie, else "
    "<cache>/menagerie)",
    "ROBOUSE_ROBOHARM_TEX": "generated texture cache for the roboharm scenes (default <cache>/roboharm_tex)",
    "ROBOUSE_PRICING": "JSON price table for trial cost (default: the packaged pricing.json)",
    # episode server
    "ROBOUSE_SOCKET": "episode server socket the `robo` client talks to (set by the runner)",
    "ROBOUSE_ORACLE_TOKEN": "per-episode token that unlocks privileged state for the reference solution (set by the "
    "runner)",
    "ROBOUSE_RECORD_SIZE": "episode video size WIDTHxHEIGHT (default: the backend's native size)",
    "ROBOUSE_EPISODE_DIR": "episode folder a task verifier reads (set by the runner)",
    "ROBOUSE_VERIFIER_DIR": "folder a task verifier writes reward.txt to (set by the runner)",
    "ROBOUSE_TEXT_ONLY": "1: `robo observe --image` prints the path without attaching the image (set for harnesses "
    "whose API rejects images)",
    # local runner sandbox
    "ROBOUSE_SANDBOX": "0 disables the macOS Seatbelt sandbox around agent processes (default 1)",
    "ROBOUSE_SANDBOX_DENY": "extra colon-separated paths agents may neither read nor write",
    # simulator worker interpreters (one virtualenv per simulator that conflicts with the main environment)
    "ROBOUSE_DEXJOCO_PYTHON": "Python of the DexJoCo virtualenv (default <cache>/dexjoco-venv/bin/python)",
    "ROBOUSE_DMCONTROL_PYTHON": "Python of the dm_control virtualenv (default <cache>/dmcontrol-venv/bin/python)",
    "ROBOUSE_HUMANOIDBENCH_PYTHON": "Python of the HumanoidBench virtualenv (default <cache>/humanoidbench-venv/bin/python)",
    "ROBOUSE_MANISKILL_PYTHON": "Python of the ManiSkill virtualenv (default <cache>/maniskill-venv/bin/python)",
    "ROBOUSE_METADRIVE_PYTHON": "Python of the MetaDrive virtualenv (default <cache>/metadrive-venv/bin/python)",
    "ROBOUSE_MYOSUITE_PYTHON": "Python of the MyoSuite virtualenv (default <cache>/myosuite-venv/bin/python)",
    "ROBOUSE_ROBOCASA_PYTHON": "Python of the RoboCasa virtualenv (default <cache>/robocasa-venv/bin/python)",
    "ROBOUSE_RLEBENCH_PYTHON": "Python of the RLE-Bench virtualenv (default <cache>/rlebench-venv/bin/python)",
    "ROBOUSE_CAPX_PYTHON": "Python of the CaP-X virtualenv (default <cache>/capx-venv/bin/python)",
    "ROBOUSE_CAPX_LIBERO_PYTHON": "Python of the CaP-X LIBERO virtualenv (default <cache>/capx-libero-venv/bin/python)",
    **{
        f"ROBOUSE_{b}_QUIET": f"0 shows the {b.lower()} worker's stderr (default 1)"
        for b in ("DEXJOCO", "DMCONTROL", "HUMANOIDBENCH", "MANISKILL", "METADRIVE", "MYOSUITE", "ROBOCASA", "RLEBENCH", "CAPX", "CAPX_LIBERO")
    },
    "ROBOUSE_MANISKILL_DEBUG": "1: the ManiSkill worker logs each request",
    "ROBOUSE_HUMANOIDBENCH_POLICY": "HumanoidBench low-level walking policy (default <cache>/humanoidbench/h1_motion.pt)",
    "ROBOUSE_ROBOSUITE_PATH": "isolated robosuite 1.5 install for the robosuite suite (default <venv>/robosuite-1.5)",
    # remote GPU workers
    "ROBOUSE_BEHAVIOR_REMOTE": "BEHAVIOR worker endpoints file (default <config>/remote/behavior.json)",
    "ROBOUSE_GPU_REMOTE": "GPU-track worker endpoints file (default <config>/remote/gpu.json)",
    "ROBOUSE_ROBODOJO_REMOTE": "RoboDojo worker endpoints file (default <config>/remote/robodojo.json)",
    "ROBOUSE_BEHAVIOR_SECRET": "shared secret the BEHAVIOR worker requires (worker side; the client reads it from the "
    "endpoints file)",
    "ROBOUSE_GPU_SECRET": "shared secret the GPU-track worker requires (worker side)",
    "ROBOUSE_ROBODOJO_SECRET": "shared secret the RoboDojo worker requires (worker side)",
    # remote worker tuning (worker side)
    "ROBOUSE_BEHAVIOR_DEBUG": "1: the BEHAVIOR worker logs each request",
    "ROBOUSE_BEHAVIOR_PLAN": "BEHAVIOR worker: motion-planning mode for skills",
    "ROBOUSE_BEHAVIOR_TORCH_THREADS": "BEHAVIOR worker: torch CPU threads",
    "ROBOUSE_BEHAVIOR_MAX_GRASP_M": "BEHAVIOR worker: largest object size the grasp skill accepts, in metres",
    "ROBOUSE_ROBODOJO_DEBUG": "1: the RoboDojo worker logs each request",
    "ROBOUSE_ROBODOJO_KIT_THREADS": "RoboDojo worker: Omniverse Kit worker threads",
    "ROBOUSE_ROBODOJO_TASK": "RoboDojo worker: task to preload at start",
    "ROBOUSE_ROBODOJO_SKILL_WALL_S": "RoboDojo worker: wall-clock limit per skill call, in seconds",
    "ROBOUSE_GPU_MAX_SESSIONS": "GPU-track worker: concurrent sessions",
    "ROBOUSE_GPU_IDLE_S": "GPU-track worker: idle seconds before a session is closed",
    "ROBOUSE_MS_SIM": "GPU-track worker: ManiSkill simulation backend",
    "ROBOUSE_PG_POLICIES": "GPU-track worker: folder of trained MuJoCo Playground policies",
    "ROBOUSE_IL_RTX": "GPU-track worker: 1 renders Isaac Lab with RTX",
    # harnesses
    "ROBOUSE_VLA_URL": "policy server for the `vla` harness",
    "ROBOUSE_VLA_TOKEN": "bearer token for the policy server",
    "ROBOUSE_VLA_POLICY_CONFIG": "JSON file with overrides for the `vla` preset",
    "ROBOUSE_VLA_PYTHON": "interpreter that runs the `vla` client (default python3)",
    "ROBOUSE_VLA_CONFIG": "resolved `vla` preset, passed to the client (set by the runner)",
    "ROBOUSE_MOLMOACT2_URL": "MolmoAct2 policy server for the `molmoact2` harness",
    "ROBOUSE_MOLMOACT2_TOKEN": "bearer token for the MolmoAct2 server",
    "ROBOUSE_TASK_LANGUAGE": "the language instruction a VLA client sends (set by the runner)",
    "ROBOUSE_GRADER_URL": "OpenAI-compatible endpoint for `robouse grade` (default: Baseten)",
    "ROBOUSE_GRADER_API_KEY": "API key for `robouse grade` (default: BASETEN_API_KEY)",
    # real robots
    "ROBOUSE_RIG": "real-robot rig file; real tasks run on live hardware instead of the hardware-in-the-loop mock",
    "ROBOUSE_OPERATOR_DIR": "operator channel and e-stop latch for real robots (default ~/.config/robouse-operator)",
    "ROBOUSE_OPERATOR": "operator channel: tty, file or script (default: tty when attended)",
    "ROBOUSE_OPERATOR_SCRIPT": "scripted operator answers (hardware-in-the-loop tests only)",
    "ROBOUSE_ESTOP_FILE": "e-stop latch file (default <operator dir>/ESTOP)",
    "ROBOUSE_REAL_UNATTENDED": "1: run real tasks without an operator (mock only)",
    "ROBOUSE_REAL_ORACLE": "reference-solution mode for real tasks (hardware-in-the-loop tests)",
    "ROBOUSE_HIL_DIR": "hardware-in-the-loop fixtures (default: the packaged fixtures)",
}

# the variables users set; the rest are set by robouse itself, belong to remote workers, or are for tests
PUBLIC = frozenset(
    {
        "ROBOUSE_CACHE_DIR",
        "ROBOUSE_CONFIG_DIR",
        "ROBOUSE_MENAGERIE_ASSETS",
        "ROBOUSE_PRICING",
        "ROBOUSE_RECORD_SIZE",
        "ROBOUSE_SANDBOX",
        "ROBOUSE_SANDBOX_DENY",
        "ROBOUSE_DEXJOCO_PYTHON",
        "ROBOUSE_DMCONTROL_PYTHON",
        "ROBOUSE_HUMANOIDBENCH_PYTHON",
        "ROBOUSE_MANISKILL_PYTHON",
        "ROBOUSE_METADRIVE_PYTHON",
        "ROBOUSE_MYOSUITE_PYTHON",
        "ROBOUSE_ROBOCASA_PYTHON",
        "ROBOUSE_RLEBENCH_PYTHON",
        "ROBOUSE_CAPX_PYTHON",
        "ROBOUSE_CAPX_LIBERO_PYTHON",
        "ROBOUSE_VLA_URL",
        "ROBOUSE_VLA_TOKEN",
        "ROBOUSE_VLA_POLICY_CONFIG",
        "ROBOUSE_MOLMOACT2_URL",
        "ROBOUSE_MOLMOACT2_TOKEN",
    }
)


def env(name: str, default: str = "") -> str:
    """The value of a declared ROBOUSE_* variable (or another process variable), or `default` when unset or empty."""
    if name.startswith("ROBOUSE_") and name not in ENV:
        raise KeyError(f"{name} is not declared in robouse.config.ENV")
    return os.environ.get(name) or default


def flag(name: str, default: bool = False) -> bool:
    """A boolean variable: 1/true/yes/on or 0/false/no/off; `default` when unset."""
    v = env(name).strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")


def path(name: str, default: Path | str) -> Path:
    """A path variable (~ expanded), or `default`."""
    return Path(env(name) or str(default)).expanduser()


def cache_dir() -> Path:
    return path("ROBOUSE_CACHE_DIR", Path.home() / ".cache" / "robouse")


def config_dir() -> Path:
    return path("ROBOUSE_CONFIG_DIR", Path.home() / ".config" / "robouse")


def operator_dir() -> Path:
    return path("ROBOUSE_OPERATOR_DIR", Path.home() / ".config" / "robouse-operator")


def sim_python(backend: str, venv: str | None = None) -> Path:
    """Interpreter of a simulator's own virtualenv: $ROBOUSE_<BACKEND>_PYTHON, else <cache>/<venv>-venv/bin/python."""
    return path(f"ROBOUSE_{backend.upper()}_PYTHON", cache_dir() / f"{venv or backend}-venv" / "bin" / "python")


def remote_endpoints(name: str) -> Path | None:
    """Endpoints file of a remote worker pool: $ROBOUSE_<NAME>_REMOTE, <config>/remote/<name>.json, or a maintainer
    fallback; None when there is none."""
    from . import _maintainer

    explicit = env(f"ROBOUSE_{name.upper()}_REMOTE")
    candidates = (
        [Path(explicit).expanduser()]
        if explicit
        else [config_dir() / "remote" / f"{name}.json", *_maintainer.remote_endpoint_files(name)]
    )
    return next((p for p in candidates if p.is_file()), None)


def sandbox_denied_paths() -> list[str]:
    """Paths agents may neither read nor write: private settings, the operator channel, maintainer folders, and
    $ROBOUSE_SANDBOX_DENY."""
    from . import _maintainer

    paths = [config_dir(), operator_dir(), *_maintainer.private_dirs()]
    out = [str(p) for p in paths if p.exists()]
    out += [p for p in env("ROBOUSE_SANDBOX_DENY").split(":") if p]
    return out
