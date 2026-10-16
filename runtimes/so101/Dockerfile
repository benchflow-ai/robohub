# Robo Use simulator runtime `so101`: the so101 suite (the SO-101 follower arm, one or two, and the Koch v1.1 low-cost
# robot arm on a small hobby desk) on MuJoCo 3.3.0, CPU only, software OpenGL (OSMesa). Robot models: MuJoCo Menagerie
# `robotstudio_so101` and `low_cost_robot_arm` (Apache-2.0), every file of both folders with their LICENSE files
# (54 files). They are not in the hub repository: they are downloaded while the image is built from the pinned
# Menagerie commit listed in assets/so101.json, and every file's SHA-256 is checked. The desk and props are BenchFlow's
# and built in code. The simulator itself runs without network.
# Build context: this folder (runtimes/so101 of a robohub checkout).
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
# import, build a one-arm and a two-arm scene and render a camera image (OSMesa) as a smoke test
RUN python -c "import robouse.session; from robouse.backends.so101 import So101Backend; \
[b.reset(0) or b.render(64, 64) for b in (So101Backend({'env': 'so101-sort-by-color'}), So101Backend({'env': 'so101-bimanual-lift-box'}), So101Backend({'env': 'koch-stack-two-cubes'}))]"

WORKDIR /episode
CMD ["robouse-sim-entry"]
