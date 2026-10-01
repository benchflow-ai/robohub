# Robo Use simulator runtime `myosuite`: MyoSuite 2.12.2 (musculoskeletal elbow, finger and hand models on MuJoCo 3.6.0,
# gymnasium 1.2.3) runs in its own Python 3.11 environment, started by the Robo Use episode server (Python 3.12) as a
# worker process. The MyoSim models ship inside the pinned myosuite wheel (its SHA-256 is checked below), so the
# simulator needs no network at run time.
# Build context: this folder (runtimes/myosuite of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python GIT_PYTHON_REFRESH=quiet

COPY requirements.txt requirements-myosuite.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt uv==0.12.5 && rm -rf /root/.cache

ARG MYOSUITE_WHEEL_SHA256=bec0133d9d4ba0249aeccdd7de632eead4749436ffb4e5a3c44a8f9cedafd9ea
RUN pip download -q --no-deps --only-binary=:all: --python-version 3.11 -d /tmp/wheels myosuite==2.12.2 \
 && echo "$MYOSUITE_WHEEL_SHA256  /tmp/wheels/myosuite-2.12.2-py3-none-any.whl" | sha256sum -c - \
 && uv venv -q -p 3.11.16 /opt/myosuite-venv \
 && VIRTUAL_ENV=/opt/myosuite-venv uv pip install -q /tmp/wheels/myosuite-2.12.2-py3-none-any.whl -r /tmp/requirements-myosuite.txt \
 && rm -rf /tmp/wheels /root/.cache
ENV ROBOUSE_MYOSUITE_PYTHON=/opt/myosuite-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.myosuite" \
 && /opt/myosuite-venv/bin/python -c "import myosuite, scipy; from myosuite.utils import gym; gym.make('myoHandPose4Fixed-v0')"

WORKDIR /episode
CMD ["robouse-sim-entry"]
