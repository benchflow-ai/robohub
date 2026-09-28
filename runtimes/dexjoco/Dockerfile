# Robo Use simulator runtime `dexjoco`: DexJoCo (Allegro hands on Franka arms) runs in its own Python 3.11 environment
# (MuJoCo 3.4.0, numpy 1.26), started by the Robo Use episode server (Python 3.12) as a worker process.
# DexJoCo (about 300 MB of scenes and meshes) is not in the hub repository: it is cloned while the image is built,
# at the pinned commit below. The simulator itself runs without network.
# Build context: this folder (runtimes/dexjoco of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates build-essential \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python

COPY requirements.txt requirements-dexjoco.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt uv==0.12.5 && rm -rf /root/.cache

ARG DEXJOCO_COMMIT=8d23b0fab23b17a58c4b55f3942e17013aaf8267
RUN git init -q /opt/dexjoco \
 && git -C /opt/dexjoco fetch -q --depth 1 https://github.com/brave-eai/dexjoco.git "$DEXJOCO_COMMIT" \
 && git -C /opt/dexjoco checkout -q FETCH_HEAD \
 && rm -rf /opt/dexjoco/.git \
 && uv venv -q -p 3.11.16 /opt/dexjoco-venv \
 && VIRTUAL_ENV=/opt/dexjoco-venv uv pip install -q -r /tmp/requirements-dexjoco.txt \
 && VIRTUAL_ENV=/opt/dexjoco-venv uv pip install -q --no-deps -e /opt/dexjoco/dexjoco \
 && (/opt/dexjoco-venv/bin/python -m compileall -q /opt/dexjoco/dexjoco || true) \
 && rm -rf /root/.cache
ENV ROBOUSE_DEXJOCO_PYTHON=/opt/dexjoco-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.dexjoco" \
 && /opt/dexjoco-venv/bin/python -c "import dexjoco"

WORKDIR /episode
CMD ["robouse-sim-entry"]
