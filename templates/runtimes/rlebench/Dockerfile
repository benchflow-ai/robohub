# Robo Use simulator runtime `rlebench`: RLE-Bench's interactive-control scenes (RLE-Bench 34a73aa, MIT) on RLE-Bench's
# pinned simulator stack: robosuite 5ce6643 and RoboCasa v1.0.1 (b4684e6) with MuJoCo 3.3.1, numpy 2.2.5 and scipy 1.15.3,
# in their own Python 3.12 environment (/opt/rlebench-venv), started by the Robo Use episode server (Python 3.12,
# MuJoCo 3.3.0) as a worker process. robosuite and RoboCasa are fetched from GitHub at the pinned commits. RLE-Bench
# renders with EGL on an NVIDIA GPU; this runtime renders on the CPU with OSMesa, which RLE-Bench supports as its
# fallback (with numba's JIT off, as RLE-Bench requires for OSMesa). The simulator itself runs without network.
# Build context: this folder (runtimes/rlebench of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa NUMBA_DISABLE_JIT=1 \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-rlebench.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

ARG ROBOSUITE_COMMIT=5ce6643f3092639d08f7b0f90ed1c6a84f50552c
ARG ROBOCASA_COMMIT=b4684e6ee37d377cc392e98302a6b916d588b415
RUN git init -q /opt/robosuite-src \
 && git -C /opt/robosuite-src fetch -q --depth 1 https://github.com/ARISE-Initiative/robosuite.git "$ROBOSUITE_COMMIT" \
 && git -C /opt/robosuite-src checkout -q FETCH_HEAD && rm -rf /opt/robosuite-src/.git /opt/robosuite-src/docs \
 && git init -q /opt/robocasa-src \
 && git -C /opt/robocasa-src fetch -q --depth 1 https://github.com/robocasa/robocasa.git "$ROBOCASA_COMMIT" \
 && git -C /opt/robocasa-src checkout -q FETCH_HEAD && rm -rf /opt/robocasa-src/.git \
 && python -m venv /opt/rlebench-venv \
 && /opt/rlebench-venv/bin/pip install --no-cache-dir --no-deps -r /tmp/requirements-rlebench.txt \
 && /opt/rlebench-venv/bin/pip install --no-cache-dir --no-deps -e /opt/robosuite-src -e /opt/robocasa-src \
 && rm -rf /root/.cache \
 && (/opt/rlebench-venv/bin/python -m compileall -q /opt/robosuite-src/robosuite /opt/robocasa-src/robocasa || true)
ENV ROBOUSE_RLEBENCH_PYTHON=/opt/rlebench-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# import check, then build and render one scene once (checks the simulator stack and the vendored RLE-Bench code)
RUN python -c "import robouse.session, robouse.backends.rlebench" \
 && cd /tmp && PYTHONPATH= /opt/rlebench-venv/bin/python -c "import runpy; \
W = runpy.run_path('/opt/robohub-runtime/robouse/src/robouse/backends/rlebench/worker.py'); \
w = W['Worker'](); print(w.make('TowerMaxHeight', 1)); print(w.render('robot0_agentview_left').shape); w.close()"

WORKDIR /episode
CMD ["robouse-sim-entry"]
