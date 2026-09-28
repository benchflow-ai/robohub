# Robo Use simulator runtime `menagerie`: the `base` runtime plus the MuJoCo Menagerie robot models (Franka Panda,
# ALOHA 2, Skydio X2) used by the menagerie, RoboHarm and drone suites. The meshes (about 43 MB) are not in the hub
# repository: they are downloaded while the image is built from the pinned Menagerie commit listed in
# assets/*.json, and every file's SHA-256 is checked. The simulator itself runs without network.
# Build context: this folder (runtimes/menagerie of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

COPY fetch_assets.py /tmp/fetch_assets.py
COPY assets/ /tmp/assets/
RUN python /tmp/fetch_assets.py /opt/menagerie /tmp/assets/*.json
ENV ROBOUSE_MENAGERIE_ASSETS=/opt/menagerie

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.menagerie, robouse.backends.roboharm, robouse.backends.drone"

WORKDIR /episode
CMD ["robouse-sim-entry"]
