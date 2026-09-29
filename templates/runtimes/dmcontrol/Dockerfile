# Robo Use simulator runtime `dmcontrol`: DeepMind's dm_control 1.0.47 (the Control Suite and dm_control.manipulation) runs
# in its own Python 3.12 virtual environment, because dm_control pins its own MuJoCo (3.14.0) while the Robo Use episode
# server uses MuJoCo 3.3.0; the episode server starts it as a worker process. The models and meshes ship inside the
# pinned dm-control wheel, so the simulator needs no network at run time. Rendering is software OpenGL (OSMesa).
# Build context: this folder (runtimes/dmcontrol of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-dmcontrol.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

RUN python -m venv /opt/dmcontrol-venv \
 && /opt/dmcontrol-venv/bin/pip install --no-cache-dir --only-binary=:all: -r /tmp/requirements-dmcontrol.txt \
 && rm -rf /root/.cache
ENV ROBOUSE_DMCONTROL_PYTHON=/opt/dmcontrol-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.dmcontrol" \
 && /opt/dmcontrol-venv/bin/python -c "from dm_control import suite, manipulation; e = suite.load('cartpole', 'swingup'); e.reset(); e.physics.render(64, 64); manipulation.load('stack_2_bricks_features', seed=0).reset()"

WORKDIR /episode
CMD ["robouse-sim-entry"]
