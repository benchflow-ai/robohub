# Robo Use simulator runtime `capx`: CaP-X's RoboSuite code-as-policy environments (CaP-X 53e9966, MIT) with its
# privileged-tier APIs, in their own Python 3.12 environment (/opt/capx-venv) started by the Robo Use episode server
# (Python 3.12, MuJoCo 3.3.0) as a worker process: CaP-X's robosuite fork (uynitsuj/robosuite 97292732, robosuite
# 1.5.1 with CaP-X's task changes) on MuJoCo 3.5.0, and PyRoKi (JAX on the CPU) for inverse kinematics, as CaP-X's
# `--profile minimal` runs it. The `panda_description` robot model PyRoKi loads is fetched while the image is built.
# No perception model (SAM3, Molmo 2, OWL-ViT, SAM2, Contact-GraspNet) is in this runtime. The simulator runs without
# network. Build context: this folder (runtimes/capx of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates build-essential \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa NUMBA_DISABLE_JIT=1 JAX_PLATFORMS=cpu \
    JAX_COMPILATION_CACHE_DIR=/opt/jax-cache JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS=0 JAX_PERSISTENT_CACHE_MIN_ENTRY_SIZE_BYTES=0 \
    ROBOT_DESCRIPTIONS_CACHE=/opt/robot_descriptions \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-capx*.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

ARG ROBOSUITE_COMMIT=97292732ed909ac3ae116579fb768607034a4dbd
RUN git init -q /opt/robosuite-src \
 && git -C /opt/robosuite-src fetch -q --depth 1 https://github.com/uynitsuj/robosuite.git "$ROBOSUITE_COMMIT" \
 && git -C /opt/robosuite-src checkout -q FETCH_HEAD && rm -rf /opt/robosuite-src/.git /opt/robosuite-src/docs \
 && python -m venv /opt/capx-venv \
 && if [ -f /tmp/requirements-capx.txt ]; then /opt/capx-venv/bin/pip install --no-cache-dir --no-deps -r /tmp/requirements-capx.txt; \
    else /opt/capx-venv/bin/pip install --no-cache-dir -r /tmp/requirements-capx-top.txt; fi \
 && /opt/capx-venv/bin/pip install --no-cache-dir --no-deps "pyroki @ git+https://github.com/chungmin99/pyroki.git@95afccc22658c461ab1042a048ae4e9c24bc2a47" \
 && /opt/capx-venv/bin/pip install --no-cache-dir --no-deps -e /opt/robosuite-src \
 && rm -rf /root/.cache \
 && /opt/capx-venv/bin/python -c "from robot_descriptions.loaders.yourdfpy import load_robot_description; print(load_robot_description('panda_description'))" \
 && chmod -R a+rX /opt/robot_descriptions \
 && git config --system --add safe.directory '*' 
ENV ROBOUSE_CAPX_PYTHON=/opt/capx-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# import check, then compile PyRoKi's IK solves once into the persistent JAX cache, so an episode starts without compiling
RUN python -c "import robouse.session, robouse.backends.capx" \
 && cd /tmp && PYTHONPATH= /opt/capx-venv/bin/python -c "import runpy; \
W = runpy.run_path('/opt/robohub-runtime/robouse/src/robouse/backends/capx/worker.py'); W['Worker']._warm_ik()" \
 && chmod -R a+rwX /opt/jax-cache

WORKDIR /episode
CMD ["robouse-sim-entry"]
