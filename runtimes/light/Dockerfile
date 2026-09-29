# Robo Use simulator runtime `light`: numpy-only simulators (Robo Use navigation homes, inspect-robots CubePick).
# Runs the trusted episode server and the verifier of a task; the agent container never uses this image.
# Build context: this folder (runtimes/light of a robohub checkout).
FROM python:3.12-slim-bookworm

# No rendering engine: these simulators draw their images with Pillow.
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

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
RUN python -c "import robouse.session, robouse.backends.nav, robouse.backends.cubepick"

WORKDIR /episode
CMD ["robouse-sim-entry"]
