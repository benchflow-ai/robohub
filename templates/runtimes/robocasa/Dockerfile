# Robo Use simulator runtime `robocasa`: RoboCasa365 kitchens on robosuite 1.5.2 and MuJoCo 3.3.1, in their own Python
# 3.12 environment (/opt/robocasa-venv), started by the Robo Use episode server (Python 3.12, MuJoCo 3.3.0) as a worker
# process. RoboCasa and robosuite are fetched from GitHub at the pinned commits below. RoboCasa's kitchen assets are not
# in the hub repository: a subset (environment textures, lightwheel fixtures and objects; 87 zips, about 1.9 GB) is
# downloaded while the image is built from two Hugging Face datasets at pinned revisions, and every zip is checked
# against its SHA-256 (assets/robocasa.json). The simulator itself runs without network.
# Build context: this folder (runtimes/robocasa of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-robocasa.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

ARG ROBOCASA_COMMIT=456174f62b89b8fca99eaaf33949c29fec9cfc2a
ARG ROBOSUITE_COMMIT=5ce6643f3092639d08f7b0f90ed1c6a84f50552c
RUN git init -q /opt/robocasa-src \
 && git -C /opt/robocasa-src fetch -q --depth 1 https://github.com/robocasa/robocasa.git "$ROBOCASA_COMMIT" \
 && git -C /opt/robocasa-src checkout -q FETCH_HEAD && rm -rf /opt/robocasa-src/.git \
 && git init -q /opt/robosuite-src \
 && git -C /opt/robosuite-src fetch -q --depth 1 https://github.com/ARISE-Initiative/robosuite.git "$ROBOSUITE_COMMIT" \
 && git -C /opt/robosuite-src checkout -q FETCH_HEAD && rm -rf /opt/robosuite-src/.git /opt/robosuite-src/docs \
 && python -m venv /opt/robocasa-venv \
 && /opt/robocasa-venv/bin/pip install --no-cache-dir --no-deps -r /tmp/requirements-robocasa.txt \
 && /opt/robocasa-venv/bin/pip install --no-cache-dir --no-deps -e /opt/robosuite-src -e /opt/robocasa-src \
 && rm -rf /root/.cache
ENV ROBOUSE_ROBOCASA_PYTHON=/opt/robocasa-venv/bin/python

# the asset subset: fetch and SHA-256-check every zip, then extract it under robocasa/models/assets/<extract_to>
COPY fetch_assets.py /tmp/robocasa-assets/
COPY assets/robocasa.json /tmp/robocasa-assets/
RUN python /tmp/robocasa-assets/fetch_assets.py /tmp/robocasa-assets/zips /tmp/robocasa-assets/robocasa.json \
 && python -c "import json, zipfile; from pathlib import Path; \
root = Path('/opt/robocasa-src/robocasa/models/assets'); \
[zipfile.ZipFile(Path('/tmp/robocasa-assets/zips') / f['path']).extractall(root / f['extract_to']) \
 for f in json.loads(Path('/tmp/robocasa-assets/robocasa.json').read_text())['files']]" \
 && rm -rf /tmp/robocasa-assets \
 && (/opt/robocasa-venv/bin/python -m compileall -q /opt/robosuite-src/robosuite /opt/robocasa-src/robocasa || true)

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# import check, then build one kitchen once (compiles robosuite's numba kernels into their cache, checks the assets)
RUN python -c "import robouse.session, robouse.backends.robocasa" \
 && cd /tmp && PYTHONPATH= /opt/robocasa-venv/bin/python -c "import runpy, sys; \
W = runpy.run_path('/opt/robohub-runtime/robouse/src/robouse/backends/robocasa/worker.py'); \
w = W['Worker'](); print(w.make('CloseDrawer', 1, 1, 0)['lang']); w.env.close()"

WORKDIR /episode
CMD ["robouse-sim-entry"]
