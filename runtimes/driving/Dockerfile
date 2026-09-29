# Robo Use simulator runtime `driving`: MetaDrive 0.4.3 (Apache-2.0; Bullet vehicle dynamics, procedural roads, IDM traffic)
# runs in its own Python 3.11 environment, started by the Robo Use episode server (Python 3.12) as a worker process
# (backends/driving_worker.py, JSON lines over stdin/stdout). Only MetaDrive's 2D top-down renderer (pygame, SDL dummy
# video driver) is used; there is no 3D window.
# MetaDrive downloads its asset pack on first import; here it is fetched while the image is built from the MetaDrive
# 0.4.3 GitHub release, checked against the SHA-256 below and unpacked where MetaDrive looks (metadrive/assets), so
# nothing is downloaded at run time. The 3D models in the pack are third-party Sketchfab models under CC-BY-4.0 (each
# model folder has a license.txt naming its author); the suite does not render them. The simulator runs without network.
# Build context: this folder (runtimes/driving of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 ca-certificates unzip \
 && rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy PYGAME_HIDE_SUPPORT_PROMPT=1

COPY requirements.txt requirements-metadrive.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt uv==0.12.5 && rm -rf /root/.cache

RUN uv venv -q -p 3.11.16 /opt/metadrive-venv \
 && VIRTUAL_ENV=/opt/metadrive-venv uv pip install -q -r /tmp/requirements-metadrive.txt \
 && rm -rf /root/.cache
ENV ROBOUSE_METADRIVE_PYTHON=/opt/metadrive-venv/bin/python

ARG METADRIVE_ASSETS_URL=https://github.com/metadriverse/metadrive/releases/download/MetaDrive-0.4.3/assets.zip
ARG METADRIVE_ASSETS_SHA256=4f0da9f5143a1258131c5b55f77bdf170c0f9bce8a9f18dd41b3678df779eac9
RUN python - <<'PY'
import hashlib, os, time, urllib.request
url, want = os.environ["METADRIVE_ASSETS_URL"], os.environ["METADRIVE_ASSETS_SHA256"]
for attempt in range(4):
    try:
        urllib.request.urlretrieve(url, "/tmp/metadrive-assets.zip")
        break
    except OSError:
        if attempt == 3:
            raise
        time.sleep(2 ** attempt)
h = hashlib.sha256(open("/tmp/metadrive-assets.zip", "rb").read()).hexdigest()
if h != want:
    raise SystemExit(f"sha256 mismatch for {url}: got {h}, pinned {want}")
print("MetaDrive assets verified:", h)
PY
RUN SITE=$(/opt/metadrive-venv/bin/python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])") \
 && unzip -q /tmp/metadrive-assets.zip -d /tmp/metadrive-assets \
 && rm -rf "$SITE/metadrive/assets" && mv /tmp/metadrive-assets/assets "$SITE/metadrive/assets" \
 && rm -rf /tmp/metadrive-assets /tmp/metadrive-assets.zip \
 && test "$(cat "$SITE/metadrive/assets/version.txt")" = "0.4.3" \
 && /opt/metadrive-venv/bin/python -c "from metadrive.engine.asset_loader import AssetLoader; assert not AssetLoader.should_update_asset(); from metadrive.envs.metadrive_env import MetaDriveEnv; import pygame, panda3d"

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
RUN python -c "import robouse.session, robouse.backends.driving"

WORKDIR /episode
CMD ["robouse-sim-entry"]
