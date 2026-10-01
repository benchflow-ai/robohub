# robouse simulator: the trusted simulator + episode server + verifier image of a robouse task on BenchFlow.
#
# Build context: the simulator runtime written by robouse.engine.format: this file, requirements.txt, a copy of the
# robouse package under robouse/src/robouse, and BenchFlow's embodied layer (benchflow_embodied/, the episode server,
# the physical verifier; embodied-sim-entry, the entry point). The agent container never uses this image.
FROM python:3.12-slim-bookworm

# Headless rendering: MuJoCo renders through OSMesa (CPU); no X server or GPU needed.
RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 \
 && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt /tmp/requirements-sim.txt
RUN pip install --no-cache-dir -r /tmp/requirements-sim.txt && rm -rf /root/.cache

# Robot meshes for the menagerie and drone suites (MuJoCo Menagerie, pinned commit, SHA-256 checked). Downloaded at
# build time; the simulator runs with network_mode: none. Only the fetch code and the manifests enter this layer.
COPY robouse/src/robouse/__init__.py robouse/src/robouse/assets.py robouse/src/robouse/config.py robouse/src/robouse/_maintainer.py /tmp/ra/robouse/
COPY robouse/src/robouse/lib/ /tmp/ra/robouse/lib/
COPY robouse/src/robouse/assets_provenance/ /tmp/ra/robouse/assets_provenance/
RUN PYTHONPATH=/tmp/ra python -c "from pathlib import Path; from robouse.assets import fetch; \
fetch(Path('/opt/robouse-assets'), log=lambda *_: None, everything=True)" && rm -rf /tmp/ra
ENV ROBOUSE_MENAGERIE_ASSETS=/opt/robouse-assets

# BenchFlow's embodied layer (vendored; BenchFlow itself is not installed here) and its entry point.
COPY benchflow_embodied/ /opt/benchflow_embodied/
COPY embodied-sim-entry /usr/local/bin/embodied-sim-entry
# robouse itself is pure Python; run it from source.
COPY robouse/ /opt/robouse/
ENV PYTHONPATH=/opt/benchflow_embodied:/opt/robouse/src
RUN chmod 755 /usr/local/bin/embodied-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier \
 && python -c "import benchflow.embodied.server, robouse.engine.sim, robouse.backends.metaworld"

WORKDIR /episode
CMD ["embodied-sim-entry"]
