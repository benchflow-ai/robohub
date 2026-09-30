# Robo Use simulator runtime `robosuite`: MuJoCo 3.3.0 and robosuite 1.5.2 (Panda, UR5e, Kinova Gen3, IIWA, Sawyer).
# Build context: this folder (runtimes/robosuite of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 build-essential \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-nodeps.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt \
 && pip install --no-cache-dir --no-deps -r /tmp/requirements-nodeps.txt \
 && rm -rf /root/.cache \
 && (python -m compileall -q /usr/local/lib/python3.12/site-packages/robosuite || true)

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "from robouse.backends.robosuite.backend import _import_robosuite; _import_robosuite()"

WORKDIR /episode
CMD ["robouse-sim-entry"]
