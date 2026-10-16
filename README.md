# robohub

BenchFlow datasets of Robo Use robotics tasks. Each dataset version is a set of native BenchFlow task packages pinned to a commit of this repository in [`registry.json`](registry.json), so `bench` runs it by name. Browse the datasets at [robouse.ai/hub](https://robouse.ai/hub/).

```sh
ROBOUSE_ORACLE_TOKEN=$(openssl rand -hex 16) \
  bench eval run \
  -d farama-foundation/metaworld@0.3 \
  --registry https://robouse.ai/hub/registry.json \
  --agent oracle
```

`ROBOUSE_ORACLE_TOKEN` is needed only for reference-solution runs (`--agent oracle`), not for agent runs.

## Datasets

A dataset is named `<org>/<name>`. A dataset adapted from another benchmark sits under that benchmark's GitHub organisation (lowercased); tasks BenchFlow designed sit under `benchflow/`, even when they run on someone else's simulator or robot models. An adapted dataset is BenchFlow's adaptation; the original authors did not publish or review it.

| Dataset | Tasks | Simulator runtime | Original benchmark |
|---|---|---|---|
| `benchflow/robouse-core@0.4` | 100 | base, menagerie, robosuite, libero, dexjoco | a sample of the suites below ([`core.txt`](core.txt)) |
| `farama-foundation/metaworld@0.3` | 50 | base | Meta-World |
| `farama-foundation/gymnasium-robotics@0.3` | 12 | base | Gymnasium-Robotics |
| `farama-foundation/franka-kitchen@0.3` | 10 | base | Franka Kitchen (Gymnasium-Robotics) |
| `farama-foundation/adroit-hand@0.3` | 12 | base | Adroit hand (Gymnasium-Robotics; D4RL demonstrations) |
| `benchflow/robouse-tabletop@0.3` | 119 | base | |
| `lifelong-robot-learning/libero@0.4` | 40 | libero | LIBERO |
| `lifelong-robot-learning/libero-90@0.3` | 90 | libero | LIBERO |
| `arise-initiative/robosuite@0.3` | 21 | robosuite | robosuite |
| `benchflow/robouse-menagerie@0.3` | 15 | menagerie | |
| `benchflow/robouse-drone@0.3` | 10 | menagerie | |
| `benchflow/robouse-harm@0.3` | 80 | menagerie | BenchFlow's reproduction of Robocurve's RoboHarm |
| `brave-eai/dexjoco@0.3` | 14 | dexjoco | DexJoCo |
| `benchflow/quadruped@0.3` | 11 | embodied | |
| `benchflow/humanoid@0.3` | 9 | embodied | |
| `benchflow/mobile-manip@0.3` | 10 | embodied | |
| `benchflow/dexhand@0.3` | 7 | embodied | |
| `benchflow/crazyflie@0.3` | 7 | embodied | |
| `benchflow/robouse-composed@0.4` | 48 | composed | |
| `benchflow/driving@0.3` | 7 | driving | |
| `google-deepmind/dm-control@0.3` | 17 | dmcontrol | dm_control |
| `myohub/myosuite@0.3` | 15 | myosuite | MyoSuite |
| `carlosferrazza/humanoid-bench@0.3` | 7 | humanoidbench | HumanoidBench |
| `mani-skill/maniskill@0.3` | 12 | maniskill | ManiSkill3 |
| `robocasa/robocasa@0.3` | 24 | robocasa | RoboCasa |
| `benchflow/robouse-nav@0.3` | 40 | light | |
| `robocurve/cubepick-reach@0.3` | 5 | light | inspect-robots cubepick-reach |
| `benchflow/robouse-noop-control@0.4` | 100 | as robouse-core | robouse-core's tasks with reference solutions that do nothing; each must score 0 |
| `stanfordvl/behavior-1k@0.4` | 43 | behavior (GPU) | BEHAVIOR-1K |
| `haosulab/maniskill-gpu@0.4` | 12 | gpu (GPU) | ManiSkill3 |
| `benchflow/robouse-genesis@0.4` | 3 | gpu (GPU) | |
| `benchflow/robouse-playground@0.4` | 5 | gpu (GPU) | |
| `isaac-sim/isaaclab-factory@0.5` | 5 | gpu (GPU) | Isaac Lab Factory |
| `robodojo-benchmark/robodojo@0.4` | 9 | robodojo (GPU) | RoboDojo |
| `rle-bench/rle-bench@0.1` | 6 | rlebench | RLE-Bench, family 03 (tabletop reasoning); tower, cantilever and balance pass at RLE-Bench quality 0.5 or more, which is Robo Use's rule (RLE-Bench has no pass mark and reports the quality as the score) |
| `capgym/cap-x@0.1` | 43 | capx, capx-libero | CaP-X: RoboSuite and LIBERO-PRO tasks as code as policy with CaP-X's privileged API (object poses from the simulator, inverse kinematics; no perception models), over many turns (not one of CaP-Bench's tiers) |

[`hub.yaml`](hub.yaml) lists each dataset's upstream repository and licence and the folder its tasks are in (`dir`). A task that appears in two datasets (for example in `robouse-core` and its suite's dataset) has byte-identical packages, so the same digest.

Earlier versions stay published and unchanged, as do the names they were published under: `robouse-core@0.1` and the other plain `robouse-*` names, `benchflow/robouse-roboharm@0.1` (now `benchflow/robouse-harm`) and `benchflow/robouse-remix@0.1` (now `benchflow/robouse-composed`). They still resolve in `bench`; new versions are published only under the current names. Every version that is not current is listed under `deprecated` in [`hub.yaml`](hub.yaml) (also in `hub.json`) with the version that replaces it and the reason; do not evaluate agents on them. The earliest versions (every 0.1, and `lifelong-robot-learning/libero@0.2`) contain their tasks' oracle tokens, which this public repository exposes; `brave-eai/dexjoco@0.2`, `carlosferrazza/humanoid-bench@0.2` and `robocasa/robocasa@0.2` each left out a task, and `benchflow/robouse-composed@0.2` has an inspect-tag judge that accepts a list of every three-digit number; the others were rebuilt from Robo Use 0.2.0, whose `robo` client waits for long skills instead of giving up after 120 s.

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
  -d benchflow/robouse-core@0.4 \
  --registry https://robouse.ai/hub/registry.json \
  --agent oracle \
  --concurrency 4
# negative control (each task must score 0)
bench eval run \
  -d benchflow/robouse-noop-control@0.4 \
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
| `haosulab/maniskill-gpu`, `benchflow/robouse-genesis`, `benchflow/robouse-playground`, `isaac-sim/isaaclab-factory` | one worker per simulator: ManiSkill3 3.0.1 (SAPIEN, ray-traced cameras), Genesis 1.4.2, MuJoCo Playground 0.2.0 (MuJoCo Warp), and for `isaac-sim/isaaclab-factory@0.5` Isaac Lab 3.0.0-EA in NVIDIA's kit-less container (`nvcr.io/nvidia/isaac-lab:3.0.0-rc1-kitless`, worker family `isaaclab3`, PhysX through OvPhysX); an NVIDIA GPU. Isaac Lab's images come from NVIDIA's RTX renderer without Kit (OVRTX) or Newton's Warp rasterizer, both of which ran on driver 595; `robo info` and each trial's `result.json` name the renderer | `ROBOUSE_GPU_REMOTE` |
| `robodojo-benchmark/robodojo` | RoboDojo (NVIDIA Isaac Sim 5.1, Isaac Lab 2.3) with its Hugging Face assets, one worker per task; an NVIDIA RTX GPU with RT cores and a driver from the 580 branch | `ROBOUSE_ROBODOJO_REMOTE` |

The worker programs are in the Robo Use package (the 0.4 versions need the release after Robo Use 0.4.1), and each file's docstring says how to start it: `robouse/backends/behavior/worker.py` (in the BEHAVIOR-1K conda environment), `robouse/backends/gpu/worker/server.py` (one process per simulator family) and `robouse/backends/robodojo/worker.py` (from a RoboDojo checkout at commit `726e9aa`, in RoboDojo's environment). The simulators they drive are installed separately, under their own terms (see [Third-party simulators and assets](#third-party-simulators-and-assets)); no prebuilt worker image is published. Versions 0.4 of `haosulab/maniskill-gpu`, `benchflow/robouse-genesis`, `benchflow/robouse-playground` and `isaac-sim/isaaclab-factory` ran with `bench` and with the Robo Use runner (see [Checks](#checks)); `isaac-sim/isaaclab-factory@0.5` (Isaac Lab 3.0) ran with `bench` from the exported packages and with the Robo Use runner; `stanfordvl/behavior-1k` and `robodojo-benchmark/robodojo` were checked by the Robo Use maintainers through the Robo Use runner from a source checkout, and have not run with `bench`. A worker can run on the machine that runs `bench`: start it on the Docker bridge's address (`server.py ... --host 172.17.0.1` on Docker Engine's default bridge) and give it as `http://host.docker.internal:<port>` in the endpoints file; the 0.4 packages map `host.docker.internal` to the host. The pinned package lists of the GPU workers' virtualenvs are in the Robo Use package under `robouse/backends/gpu/worker/locks/`.

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
| `rlebench` | robosuite at `5ce6643` and RoboCasa at `b4684e6` (RLE-Bench's pins), pinned pip packages |
| `capx` | CaP-X's robosuite fork at `9729273`, PyRoKi at `95afccc`, jaxls at `6fe7cf9`, pinned pip packages (a complete lock), the `panda_description` robot model |
| `capx-libero` | LIBERO-PRO with its assets at `5368540`, CaP-X's robosuite 1.4 fork at `a498b08`, PyRoKi at `95afccc`, CPU PyTorch 2.8.0, pinned pip packages (a complete lock), the `panda_description` robot model |
| `behavior`, `gpu`, `robodojo` | a thin client; the simulators run on remote GPU workers |

## Third-party simulators and assets

The simulators, robot models, scenes and datasets the images download, or that run on remote GPU workers, are not in this repository. When you build or run them you accept each upstream's terms. Some are restrictive: ManiSkill3's assets are CC BY-NC 4.0; the BEHAVIOR-1K Data Bundle allows non-commercial academic research only and may not be redistributed; Isaac Sim, the Isaac Lab container's OvPhysX and OVRTX, and NVIDIA's Factory assets are under NVIDIA's licences (OVRTX's licence does not allow publishing benchmark or performance data about it without NVIDIA's permission); RoboCasa's assets are CC BY 4.0. RoboDojo's own terms apply to its assets, which its workers fetch at build time. Do not publish built images of the `maniskill`, `behavior`, `gpu` or `robodojo` runtimes.

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
scripts/mujoco_regress.py   a MuJoCo release -> reference and no-op verdicts of every MuJoCo-based task, diffed against the pins
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

## MuJoCo regression check

`scripts/mujoco_regress.py` re-runs every MuJoCo-based task's reference solution and no-op on a new MuJoCo release, with only MuJoCo changed, and compares the verdicts with the pinned runtime's. The MuJoCo-based runtimes pin seven versions: 3.1.6 (`humanoidbench`), 3.3.0 (`base` and most others), 3.3.1 (`robocasa`, `rlebench`), 3.4.0 (`dexjoco`), 3.5.0 (`capx`, `capx-libero`), 3.6.0 (`myosuite`) and 3.14.0 (`dmcontrol`). `light`, `driving` and `maniskill` pin MuJoCo but their physics is not MuJoCo; the GPU runtimes have no MuJoCo pin.

Each job runs on a throwaway Linux machine as root (the tool replays the runtime's Dockerfile onto it). It runs every task of its share on the pinned runtime, then switches MuJoCo in every interpreter of the runtime (`uv pip install mujoco==VERSION`, which adds only what the new MuJoCo needs and records the change) and runs them again. For the tasks whose verdict changed, it runs the pinned version once more (repeatability), seeds 1 and 2 on both versions for up to eight of them, and a bisection over the MuJoCo releases in between (four tasks in full, up to twelve more on both sides of each boundary found). Trials run as many at a time as the machine's memory holds simulators of the runtime (`hub.yaml` `sim_memory`), and a trial killed for memory is run again alone. A trial reproduces the task's BenchFlow topology without Docker: the simulator service's command and environment from `environment/docker-compose.yaml`, then `oracle/solve.sh` or the no-op (`robo info; robo observe`, then exit without `robo done`, as in `benchflow/robouse-noop-control`), then `verifier/test.sh`.

```sh
python3 scripts/mujoco_regress.py plan --mujoco 3.16.0 --out plan.json --shards base=3,menagerie=2,libero=3
python3 scripts/mujoco_regress_daytona.py --plan plan.json --results results --parallel 10 \
    --label KEY=VALUE --ledger sandboxes.jsonl            # Daytona CPU sandboxes, DAYTONA_API_KEY set
python3 scripts/mujoco_regress.py diff --plan plan.json --results results/* --out diff.json
python3 scripts/mujoco_regress.py report --diff diff.json --notes notes.json --out REPORT.md
python3 -m unittest scripts/test_mujoco_regress.py      # the tool's own tests
```

`diff.json` has, per task, both verdicts, outcomes and steps, the error and the file it was raised in, the first diverging step (from the episode's physics samples, else its robo trace), the repeat and seed checks, the bisection, and a first classification: our pin, simulator change or task fragility. `--notes` replaces that classification with reviewed attributions. `variant RUNTIME --mujoco VERSION --out DIR` writes the same switch as a copy of a runtime folder whose Dockerfile ends with it, for a Docker build. Slow suites can run the new version first and the pinned one only where the verdict moved (`job --pinned-only-changed`; the rest keep the published check as their baseline).

## Checks

Run on 2026-10-01 with the released `bench` 0.7.4 on a 16-core linux/arm64 cloud VM (Docker 29.1), through `bench eval run -d NAME@VERSION` against this repository's `registry.json`, so every run first verified all task digests at the pinned commit.

| Check | Result |
|---|---|
| `bench tasks check --sandbox docker`, every task package | 969/969 valid |
| Reference solutions, every task of the 27 CPU datasets | every task scored 1 |
| No-op reference solutions (`scripts/export.py --noop-out`), every task of every CPU dataset, and `benchflow/robouse-noop-control@0.4` | every task scored 0 |
| `rle-bench/rle-bench@0.1` and `capgym/cap-x@0.1`, exported from Robo Use 0.3.0 (run 2026-10-01, same VM and `bench`): `bench tasks check`, reference solutions, no-op reference solutions | 49/49 valid; every task scored 1 (6/6, 43/43); every task scored 0 (0/6, 0/43) |
| A scripted agent looking for the oracle token in a vision task and a composed task, with a token set on the host (0.2 packages; the token handling has not changed since) | no token in its environment, files or processes; none of about 5,400 candidate strings, including the token an earlier version committed, unlocked privileged state; a control with the reference solution's token did |
| `haosulab/maniskill-gpu@0.4`, `benchflow/robouse-genesis@0.4`, `benchflow/robouse-playground@0.4` and `isaac-sim/isaaclab-factory@0.4` (not published; 0.5 replaces it), exported from Robo Use code after 0.4.1 (run 2026-10-14 with `bench` 0.7.8 on x86_64 Daytona sandboxes with an NVIDIA RTX PRO 6000 and driver 595.91, the GPU workers on the same machine, through registry entries pinned to the export commit): reference solutions, and no-op reference solutions (`scripts/export.py --noop-out`) | every task scored 1 (12/12, 3/3, 5/5, 5/5); every task scored 0 (0/12, 0/3, 0/5, 0/5). Isaac Lab drew its images with the MuJoCo schematic (Isaac Sim's RTX renderer crashes on driver 595), as each trial's `result.json` records |
| `isaac-sim/isaaclab-factory@0.5`, the same tasks on Isaac Lab 3.0.0-EA in NVIDIA's kit-less container (Robo Use runner from the Robo Use commit it was exported from, 2026-10-16, a Daytona sandbox with an NVIDIA RTX PRO 6000 and driver 595.84, the worker in the container on the same machine): reference solutions and the no-op control, once with OVRTX images and once with Newton's renderer | every task scored 1 (5/5, twice); every task scored 0 (0/5, twice); each trial's `result.json` names the renderer. Newton physics was not run: Isaac Lab 3.0.0-EA has no Newton configuration for Factory |
| `isaac-sim/isaaclab-factory@0.5` with `bench` 0.7.8 (2026-10-16, the same sandbox, the exported packages through `--tasks-dir` before registry entries existed, the worker on the Docker bridge's address): reference solutions, and no-op reference solutions (`scripts/export.py --noop-out`) | every task scored 1 (5/5); every task scored 0 (0/5); OVRTX images |
| The composed inspect-tag judge, with the robot driven to the tag and an answer that lists every three-digit number | 1 on `benchflow/robouse-composed@0.2`, 0 on 0.3 |

Every dataset comes from a Robo Use release as published on PyPI, and every file the runtimes copy is identical to that release's wheel: `rle-bench/rle-bench@0.1` and `capgym/cap-x@0.1` come from 0.3.0, every other dataset from 0.2.0, except the six GPU datasets' 0.4 versions, which on this branch come from Robo Use code after 0.4.1 (branch int-1014/gpu) and are to be exported again from the next release before they are pinned in `registry.json`. `brave-eai/dexjoco@0.3` and `carlosferrazza/humanoid-bench@0.3` were run in full when they were published; the 0.2.0 export left their packages unchanged. The GPU datasets' versions before 0.4 were not run in the hub's checks; their packages passed `bench tasks check` only, and they cannot start under bench (their thin-client runtime has no MuJoCo, which the episode server of Robo Use 0.2.0 required).
