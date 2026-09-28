# Robo Use simulator runtime `base`: MuJoCo 3.3.0, Meta-World, Gymnasium-Robotics and the Robo Use tabletop scenes.
# Runs the trusted episode server and the verifier of a task; the agent container never uses this image.
# Build context: this folder (runtimes/base of a robohub checkout).
FROM python:3.12-slim-bookworm

# Headless rendering: MuJoCo renders through OSMesa (CPU); no X server or GPU needed.
RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

# Everything below is checked against the digest the task pins.
ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.metaworld_backend, robouse.backends.tabletop, robouse.backends.gymrobotics"

WORKDIR /episode
CMD ["robouse-sim-entry"]
