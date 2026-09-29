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
# build time; the simulator runs with network_mode: none. Only the provenance manifests enter this layer's context.
COPY robouse/src/robouse/assets_provenance/ /opt/robouse-assets-provenance/
COPY robouse/src/robouse/assets.py /tmp/robouse_assets.py
RUN python -c "import sys, pathlib; sys.path.insert(0, '/tmp'); import robouse_assets as a; \
a.manifests = lambda: sorted(pathlib.Path('/opt/robouse-assets-provenance').glob('*.json')); \
a.fetch(pathlib.Path('/opt/robouse-assets'), log=lambda *_: None)"
ENV ROBOUSE_MENAGERIE_ASSETS=/opt/robouse-assets

# BenchFlow's embodied layer (vendored; BenchFlow itself is not installed here) and its entry point.
COPY benchflow_embodied/ /opt/benchflow_embodied/
COPY embodied-sim-entry /usr/local/bin/embodied-sim-entry
# robouse itself is pure Python; run it from source.
COPY robouse/ /opt/robouse/
ENV PYTHONPATH=/opt/benchflow_embodied:/opt/robouse/src
RUN chmod 755 /usr/local/bin/embodied-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier \
 && python -c "import benchflow.embodied.server, robouse.embodied, robouse.backends.metaworld_backend"

WORKDIR /episode
CMD ["embodied-sim-entry"]
