# Robo Use simulator runtime `libero`: LIBERO (hf-libero 0.1.4) on robosuite 1.4.0 and MuJoCo 3.3.0, CPU PyTorch.
# LIBERO's scene assets (about 400 MB) are not in the hub repository: they are downloaded while the image is built,
# from the Hugging Face dataset lerobot/libero-assets at a pinned revision. The simulator itself runs without network.
# Build context: this folder (runtimes/libero of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt \
 && pip install --no-cache-dir torch==2.14.0 --index-url https://download.pytorch.org/whl/cpu \
 && pip install --no-cache-dir --no-deps hf-libero==0.1.4 robosuite==1.4.0 bddl==1.0.1 \
 && rm -rf /root/.cache \
 && (python -m compileall -q /usr/local/lib/python3.12/site-packages/libero /usr/local/lib/python3.12/site-packages/robosuite || true)

ARG LIBERO_ASSETS_REVISION=0b3ea86be5fe169d0fd036ae63d1070ec09e90f6
RUN python -c "from huggingface_hub import snapshot_download; snapshot_download('lerobot/libero-assets', repo_type='dataset', revision='$LIBERO_ASSETS_REVISION', local_dir='/root/.cache/libero/assets')" \
 && rm -rf /root/.cache/libero/assets/.cache /root/.cache/huggingface

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src HF_HUB_OFFLINE=1
# writes ~/.libero/config.yaml and imports LIBERO once, so the first episode starts quickly
RUN python -c "from robouse.backends.libero.backend import _quiet_imports; _quiet_imports()"

WORKDIR /episode
CMD ["robouse-sim-entry"]
