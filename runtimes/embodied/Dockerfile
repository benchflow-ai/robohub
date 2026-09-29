# Robo Use simulator runtime `embodied`: the embodiment suites (quadrupeds, humanoids, mobile manipulators, dexterous
# hands, Crazyflie swarm) on MuJoCo 3.3.0 with 14 MuJoCo Menagerie robots: Unitree Go2, Go1, G1, H1, Boston Dynamics
# Spot, ANYbotics ANYmal C (BSD-3-Clause); Apptronik Apollo, Booster T1, Hello Robot Stretch 3, PAL TIAGo, Google Robot,
# Shadow Hand (Apache-2.0); LEAP Hand, Bitcraze Crazyflie 2 (MIT). Each robot's LICENSE file is fetched with it.
# The models (581 files) are not in the hub repository: they are downloaded while the image is built from the pinned
# Menagerie commit listed in assets/embodiments.json, and every file's SHA-256 is checked. The simulator itself runs
# without network.
# Build context: this folder (runtimes/embodied of a robohub checkout).
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
RUN python -c "import robouse.session, robouse.backends.quadruped, robouse.backends.humanoid, robouse.backends.mobile_manip, robouse.backends.dexhand, robouse.backends.crazyflie"

WORKDIR /episode
CMD ["robouse-sim-entry"]
