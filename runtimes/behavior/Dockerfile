# Robo Use simulator runtime `behavior`: the thin client for BEHAVIOR-1K. The simulator itself (OmniGibson 3.9.3 on
# Isaac Sim 5.1) needs an NVIDIA RTX GPU and runs as remote workers (Robo Use docs/suites/behavior.md); this image runs
# the trusted episode server, which forwards reset/observe/skill/render/success calls to the worker over HTTPS.
# Build context: this folder (runtimes/behavior of a robohub checkout).
FROM python:3.12-slim-bookworm
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
RUN python -c "import robouse.session, robouse.backends.behavior"

WORKDIR /episode
CMD ["robouse-sim-entry"]
