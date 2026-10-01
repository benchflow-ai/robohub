# robohub

BenchFlow datasets of Robo Use robotics tasks. Each dataset version is a set of native BenchFlow task packages pinned to a commit of this repository in [`registry.json`](registry.json), so `bench` runs it by name. Browse the datasets at [robouse.ai/hub](https://robouse.ai/hub/).

```sh
ROBOUSE_ORACLE_TOKEN=$(openssl rand -hex 16) \
  bench eval run \
  -d farama-foundation/metaworld@0.2 \
  --registry https://robouse.ai/hub/registry.json \
  --agent oracle
```

`ROBOUSE_ORACLE_TOKEN` is needed only for reference-solution runs (`--agent oracle`), not for agent runs.

## Datasets

A dataset is named `<org>/<name>`. A dataset adapted from another benchmark sits under that benchmark's GitHub organisation (lowercased); tasks BenchFlow designed sit under `benchflow/`, even when they run on someone else's simulator or robot models. An adapted dataset is BenchFlow's adaptation; the original authors did not publish or review it.

| Dataset | Tasks | Simulator runtime | Original benchmark |
|---|---|---|---|
| `benchflow/robouse-core@0.3` | 100 | base, menagerie, robosuite, libero, dexjoco | a sample of the suites below ([`core.txt`](core.txt)) |
| `farama-foundation/metaworld@0.2` | 50 | base | Meta-World |
| `farama-foundation/gymnasium-robotics@0.2` | 12 | base | Gymnasium-Robotics |
| `farama-foundation/franka-kitchen@0.2` | 10 | base | Franka Kitchen (Gymnasium-Robotics) |
| `farama-foundation/adroit-hand@0.2` | 12 | base | Adroit hand (Gymnasium-Robotics; D4RL demonstrations) |
| `benchflow/robouse-tabletop@0.2` | 119 | base | |
| `lifelong-robot-learning/libero@0.3` | 40 | libero | LIBERO |
| `lifelong-robot-learning/libero-90@0.2` | 90 | libero | LIBERO |
| `arise-initiative/robosuite@0.2` | 21 | robosuite | robosuite |
| `benchflow/robouse-menagerie@0.2` | 15 | menagerie | |
| `benchflow/robouse-drone@0.2` | 10 | menagerie | |
| `benchflow/robouse-harm@0.2` | 80 | menagerie | BenchFlow's reproduction of Robocurve's RoboHarm |
| `brave-eai/dexjoco@0.3` | 14 | dexjoco | DexJoCo |
| `benchflow/quadruped@0.2` | 11 | embodied | |
| `benchflow/humanoid@0.2` | 9 | embodied | |
| `benchflow/mobile-manip@0.2` | 10 | embodied | |
| `benchflow/dexhand@0.2` | 7 | embodied | |
| `benchflow/crazyflie@0.2` | 7 | embodied | |
| `benchflow/robouse-composed@0.3` | 48 | composed | |
| `benchflow/driving@0.2` | 7 | driving | |
| `google-deepmind/dm-control@0.2` | 17 | dmcontrol | dm_control |
| `myohub/myosuite@0.2` | 15 | myosuite | MyoSuite |
| `carlosferrazza/humanoid-bench@0.3` | 7 | humanoidbench | HumanoidBench |
| `mani-skill/maniskill@0.2` | 12 | maniskill | ManiSkill3 |
| `robocasa/robocasa@0.2` | 23 | robocasa | RoboCasa |
| `benchflow/robouse-nav@0.2` | 40 | light | |
| `robocurve/cubepick-reach@0.2` | 5 | light | inspect-robots cubepick-reach |
| `benchflow/robouse-noop-control@0.3` | 100 | as robouse-core | robouse-core's tasks with reference solutions that do nothing; each must score 0 |
| `stanfordvl/behavior-1k@0.2` | 43 | behavior (GPU) | BEHAVIOR-1K |
| `haosulab/maniskill-gpu@0.2` | 12 | gpu (GPU) | ManiSkill3 |
| `benchflow/robouse-genesis@0.2` | 3 | gpu (GPU) | |
| `benchflow/robouse-playground@0.2` | 5 | gpu (GPU) | |
| `isaac-sim/isaaclab-factory@0.2` | 5 | gpu (GPU) | Isaac Lab Factory |
| `robodojo-benchmark/robodojo@0.2` | 9 | robodojo (GPU) | RoboDojo |

[`hub.yaml`](hub.yaml) lists each dataset's upstream repository and licence and the folder its tasks are in (`dir`). A task that appears in two datasets (for example in `robouse-core` and its suite's dataset) has byte-identical packages, so the same digest.

Earlier versions stay published and unchanged, as do the names they were published under: `robouse-core@0.1` and the other plain `robouse-*` names, `benchflow/robouse-roboharm@0.1` (now `benchflow/robouse-harm`) and `benchflow/robouse-remix@0.1` (now `benchflow/robouse-composed`). They still resolve in `bench`; new versions are published only under the current names. Do not evaluate agents on the versions listed under `deprecated` in [`hub.yaml`](hub.yaml) (also in `hub.json`), each with its reason and replacement: the earlier versions (every 0.1, and `lifelong-robot-learning/libero@0.2`) contain their tasks' oracle tokens, which this public repository exposes; `brave-eai/dexjoco@0.2` and `carlosferrazza/humanoid-bench@0.2` each left out a task, and `benchflow/robouse-composed@0.2` has an inspect-tag judge that accepts a list of every three-digit number; `benchflow/robouse-core@0.2` and `benchflow/robouse-noop-control@0.2` are superseded by 0.3, whose five DexJoCo tasks run on the newer runtime.

## Registry

- `https://raw.githubusercontent.com/benchflow-ai/robohub/main/registry.json`
- `https://robouse.ai/hub/registry.json` (a copy of the same file)

The schema is BenchFlow's dataset registry: a list of `{name, version, description, git_tag, bench_version, tasks: [{name, git_url, git_commit_id, path, digest}]}`, with `bench_version` `>=0.7.4,<0.8`. `bench` clones this repository at the pinned commit and recomputes every task's content digest before anything runs. Published versions are never changed: a changed task means a new dataset version.

## Running

Requirements: Docker with Compose v2.24.4 or newer (the task compose files use `!override`) and `bench` 0.7.4 (the registry entries require `>=0.7.4,<0.8`). On a Mac, Colima works; run `bench` from a folder under your home folder, because `bench` keeps the cloned snapshot in `.cache/datasets` of the current folder and Colima shares only the home folder with its VM.

```sh
# reference solutions (each task must score 1)
ROBOUSE_ORACLE_TOKEN=$(openssl rand -hex 16) \
  bench eval run \
  -d benchflow/robouse-core@0.3 \
  --registry https://robouse.ai/hub/registry.json \
  --agent oracle \
  --concurrency 4
# negative control (each task must score 0)
bench eval run \
  -d benchflow/robouse-noop-control@0.3 \
  --registry https://robouse.ai/hub/registry.json \
  --agent oracle
```

A task runs as two containers. `main` is BenchFlow's agent container with only the `robo` client. `simulator` is trusted, has no network, runs the Robo Use episode server and the verifier, and shares only a Unix socket with `main`. The reward is 1 if the episode server judged the task solved from the physical state, else 0.

Oracle token. In vision tasks the agent sees camera images, not object positions; the reference solution reads the true state with a token. No package contains a token. The simulator takes it from `ROBOUSE_ORACLE_TOKEN` in the environment of the `bench` process, and the reference solution gets the same variable through `oracle.env` in `task.md`, which BenchFlow passes to the oracle only. Set a fresh random value for reference-solution runs, as above. Without it the simulator has no token, nothing is privileged, and reference solutions that need the true state score 0. Agents never receive it.

The datasets marked GPU need simulator workers on a GPU machine; see [GPU datasets](#gpu-datasets).

## GPU datasets

Six datasets run their simulators on NVIDIA GPUs: `stanfordvl/behavior-1k`, `haosulab/maniskill-gpu`, `benchflow/robouse-genesis`, `benchflow/robouse-playground`, `isaac-sim/isaaclab-factory` and `robodojo-benchmark/robodojo`. Their task packages hold only a thin client: the `simulator` service (runtime `behavior`, `gpu` or `robodojo`) keeps network access and sends each request to simulator workers that you run on a GPU machine. The host running `bench` names a JSON file with the workers' endpoints and shared secret in an environment variable, which the service mounts read-only; the agent container never sees it.

| Datasets | Simulator workers and hardware | Endpoints file |
|---|---|---|
| `stanfordvl/behavior-1k` | BEHAVIOR-1K (OmniGibson 3.9.3 on NVIDIA Isaac Sim 5.1) with the BEHAVIOR Data Bundle, one worker per activity; an NVIDIA RTX GPU with RT cores and a driver from the 580 branch (Isaac Sim 5.1 crashed at start-up on 595) | `ROBOUSE_BEHAVIOR_REMOTE` |
| `haosulab/maniskill-gpu`, `benchflow/robouse-genesis`, `benchflow/robouse-playground`, `isaac-sim/isaaclab-factory` | one worker per simulator: ManiSkill3 3.0.1 (SAPIEN, ray-traced cameras), Genesis 1.4.2, MuJoCo Playground 0.2.0 (MuJoCo Warp), Isaac Lab 2.3 on Isaac Sim 5.1; an NVIDIA RTX GPU (Isaac Lab ran without its RTX renderer on driver 595) | `ROBOUSE_GPU_REMOTE` |
| `robodojo-benchmark/robodojo` | RoboDojo (NVIDIA Isaac Sim 5.1, Isaac Lab 2.3) with its Hugging Face assets, one worker per task; an NVIDIA RTX GPU with RT cores and a driver from the 580 branch | `ROBOUSE_ROBODOJO_REMOTE` |

The worker programs are in the Robo Use package (`pip install robouse==0.2.0`), and each file's docstring says how to start it: `robouse/backends/behavior/worker.py` (in the BEHAVIOR-1K conda environment), `robouse/backends/gpu/worker/server.py` (one process per simulator family) and `robouse/backends/robodojo/worker.py` (from a RoboDojo checkout at commit `726e9aa`, in RoboDojo's environment). The simulators they drive are installed separately, under their own terms (see [Third-party simulators and assets](#third-party-simulators-and-assets)); no prebuilt worker image is published. The Robo Use maintainers checked every task of these datasets with the reference solutions and a no-op control through the Robo Use runner from a source checkout; they were not run in the hub's checks, where their packages passed `bench tasks check` only.

## Simulator images

The simulator images are built from this repository, not pulled from a registry. Each task's `environment/docker-compose.yaml` builds its `simulator` service from one of the folders in `runtimes/`, and, except for `composed`, passes that folder's content digest as a build argument; the build fails if the folder does not match. So a task's registry digest also pins the simulator it runs on. The first run of a dataset builds each image it needs once; later trials use the Docker build cache.

| Runtime | What the build downloads |
|---|---|
| `base` | pinned pip packages (MuJoCo, Meta-World, Gymnasium-Robotics) |
| `light` | pinned pip packages (numpy only) |
| `menagerie` | pinned pip packages; 93 MuJoCo Menagerie files from commit `8161bba`, each checked against its SHA-256 |
| `robosuite` | pinned pip packages (robosuite 1.5.2) |
| `libero` | pinned pip packages, CPU PyTorch, LIBERO assets from the Hugging Face dataset `lerobot/libero-assets` at a pinned revision |
| `dexjoco` | pinned pip packages, DexJoCo at commit `8d23b0f` |
| `embodied` | pinned pip packages; 581 MuJoCo Menagerie files from commit `8161bba`, each checked against its SHA-256 |
| `composed` | pinned pip packages; MuJoCo Menagerie files, each checked against its SHA-256 |
| `driving` | pinned pip packages, MetaDrive 0.4.3 and its asset pack (SHA-256 pinned) |
| `dmcontrol` | pinned pip packages, dm_control 1.0.47 |
| `myosuite` | pinned pip packages, MyoSuite 2.12.2 (wheel SHA-256 pinned) |
| `humanoidbench` | pinned pip packages, HumanoidBench at commit `cb11890`, Unitree's H1 walking policy (SHA-256 pinned) |
| `maniskill` | ManiSkill 3.0.1 and SAPIEN 3.0.3 (SHA-256 pinned wheel) |
| `robocasa` | RoboCasa at `456174f`, robosuite at `5ce6643`, 87 asset archives from two Hugging Face datasets at pinned revisions, each SHA-256 checked |
| `behavior`, `gpu`, `robodojo` | a thin client; the simulators run on remote GPU workers |

## Third-party simulators and assets

The simulators, robot models, scenes and datasets the images download, or that run on remote GPU workers, are not in this repository. When you build or run them you accept each upstream's terms. Some are restrictive: ManiSkill3's assets are CC BY-NC 4.0; the BEHAVIOR-1K Data Bundle allows non-commercial academic research only and may not be redistributed; Isaac Sim and NVIDIA's Factory assets are under NVIDIA's licences; RoboCasa's assets are CC BY 4.0. RoboDojo's own terms apply to its assets, which its workers fetch at build time. Do not publish built images of the `maniskill`, `behavior`, `gpu` or `robodojo` runtimes.

## Licence

Apache-2.0 ([`LICENSE`](LICENSE)) for what BenchFlow wrote here, including the vendored copies of Robo Use and BenchFlow's embodied layer. Third-party material in this repository (Meta-World's scripted policies, LIBERO, D4RL and DexJoCo demonstration actions, MyoSuite policies, a port of inspect-robots' CubePick, renders of MuJoCo Menagerie models) keeps its own licence; [`NOTICE`](NOTICE) lists each item with its source, licence and changes.

## Layout

```
registry.json               the dataset registry (generated)
hub.json                    the index behind robouse.ai/hub (generated)
hub.yaml                    datasets, runtimes, and per-suite facts
core.txt                    the task list of robouse-core
export.json                 what was exported: Robo Use commit, runtime digests, task -> suite/backend/runtime
dogfood.json                reduced results of the bench runs of the 0.1 versions (generated by scripts/summarize_jobs.py)
datasets/<dir>/<task>/      native BenchFlow task packages (task.md, environment/, verifier/, oracle/)
runtimes/<name>/            simulator image build contexts
templates/                  the templates scripts/export.py writes packages and runtimes from
scripts/export.py           Robo Use checkout -> runtimes/ and datasets/
scripts/build_registry.py   datasets at a commit -> registry.json and hub.json
scripts/summarize_jobs.py   bench job folders -> dogfood.json
```

## Regenerating

```sh
# 1. export from a Robo Use checkout, with a Python that has Robo Use's dependencies, Meta-World and BenchFlow
python scripts/export.py --robouse ../robouse
git add -A && git commit -m "Export Robo Use <commit> as <datasets>"
# 2. pin the new dataset versions to that commit (run with the Python that has BenchFlow installed)
python scripts/build_registry.py --tag
```

To change a dataset, bump its version in `hub.yaml`, export, commit and pin; `build_registry.py` keeps every entry already in `registry.json` and pins only versions that are new.

## Checks

Run with the released `bench` 0.7.4 on a 16-core linux/arm64 cloud VM (Docker 29.1), through `bench eval run -d NAME@VERSION` against this repository's `registry.json`, so every run first verified all task digests at the pinned commit: the 0.2 datasets on 2026-09-30, the 0.3 datasets on 2026-10-01.

| Check | Result |
|---|---|
| `bench tasks check --sandbox docker`, every task package | 968/968 valid |
| Reference solutions, every task of the 27 CPU datasets | every task scored 1 (trials that first failed to start with Docker's `No such container` race scored 1 in another run with the same task digest) |
| No-op reference solutions (`scripts/export.py --noop-out`), every task of every CPU dataset, and `benchflow/robouse-noop-control@0.3` | every task scored 0 |
| A scripted agent looking for the oracle token in a vision task and a composed task, with a token set on the host | no token in its environment, files or processes; none of about 5,400 candidate strings, including the token an earlier version committed, unlocked privileged state; a control with the reference solution's token did |
| The composed inspect-tag judge, with the robot driven to the tag and an answer that lists every three-digit number | 1 on `benchflow/robouse-composed@0.2`, 0 on 0.3 |

The 0.2 datasets ran in full before their last re-export, which changed only each runtime's provenance file and the digest each task pins, and one task per dataset after it. The 0.3 datasets come from Robo Use 0.2.0 as published on PyPI: every file their runtimes copy is identical to the 0.2.0 wheel. The GPU datasets cannot run in `bench`; they were checked with `bench tasks check` only.
