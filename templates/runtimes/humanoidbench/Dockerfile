# Robo Use simulator runtime `humanoidbench`: HumanoidBench (Unitree H1 with two Shadow hands, MuJoCo) runs in its own
# Python 3.11 environment with HumanoidBench's pins (mujoco 3.1.6, gymnasium 0.29.1, dm_control 1.0.20, numpy 1.26),
# started by the Robo Use episode server (Python 3.12) as a worker process.
# HumanoidBench (MIT; about 260 MB of robot and scene assets: Unitree H1/G1 BSD-3-Clause, Shadow Hand Apache-2.0) is not in
# the hub repository: it is cloned while the image is built, at the pinned commit below; its training code and
# pretrained reaching policies are removed. The walking controller's weights (Unitree's pretrained H1 locomotion policy,
# unitree_rl_gym, BSD-3-Clause) are downloaded from a pinned commit listed in assets/humanoidbench.json, SHA-256 checked.
# No torch or jax: the worker reads the policy's TorchScript file itself and runs it in numpy. The simulator runs
# without network. Rendering is software OpenGL (OSMesa).
# Build context: this folder (runtimes/humanoidbench of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python

COPY requirements.txt requirements-humanoidbench.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt uv==0.12.5 && rm -rf /root/.cache

ARG HUMANOIDBENCH_COMMIT=cb1189039151c8aadaaa987b442da54383c87fab
RUN git init -q /opt/humanoid-bench \
 && git -C /opt/humanoid-bench fetch -q --depth 1 https://github.com/carlosferrazza/humanoid-bench.git "$HUMANOIDBENCH_COMMIT" \
 && git -C /opt/humanoid-bench checkout -q FETCH_HEAD \
 && cd /opt/humanoid-bench && rm -rf .git data dreamerv3 jaxrl_m ppo tdmpc2 logs ./*.jpg ./*.png \
 && uv venv -q -p 3.11.16 /opt/humanoidbench-venv \
 && VIRTUAL_ENV=/opt/humanoidbench-venv uv pip install -q -r /tmp/requirements-humanoidbench.txt \
 && VIRTUAL_ENV=/opt/humanoidbench-venv uv pip install -q --no-deps -e /opt/humanoid-bench \
 && rm -rf /root/.cache
ENV ROBOUSE_HUMANOIDBENCH_PYTHON=/opt/humanoidbench-venv/bin/python

COPY fetch_assets.py /tmp/fetch_assets.py
COPY assets/ /tmp/assets/
RUN python /tmp/fetch_assets.py /opt/humanoidbench-policy /tmp/assets/*.json
ENV ROBOUSE_HUMANOIDBENCH_POLICY=/opt/humanoidbench-policy/h1_motion.pt

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# smoke test: the worker builds a HumanoidBench scene, loads the walking policy and renders a frame
RUN python -c "import robouse.session, robouse.backends.humanoidbench" \
 && cd /opt/robohub-runtime/robouse/src/robouse/backends/humanoidbench \
 && printf '%s\n' '{"cmd": "make", "task": "walk", "seed": 0, "policy": "/opt/humanoidbench-policy/h1_motion.pt"}' \
      '{"cmd": "step", "action": [1, 0, 0, 0, 0, 0, 0, 0, 0]}' '{"cmd": "close"}' \
    | /opt/humanoidbench-venv/bin/python worker.py | grep -c '"ok": true' | grep -qx 3

WORKDIR /episode
CMD ["robouse-sim-entry"]
