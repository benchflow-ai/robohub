# Robo Use simulator runtime `maniskill`: ManiSkill3 (mani-skill 3.0.1 on SAPIEN 3.0.3, CPU torch 2.14.0) runs in its own
# Python 3.12 environment, started by the Robo Use episode server (Python 3.12) as a worker process. Physics runs on
# ManiSkill's CPU backend (PhysX on the CPU) and SAPIEN renders through Vulkan on Mesa's lavapipe software rasteriser, so
# no GPU is needed. Debian trixie (Mesa 25) is used because SAPIEN needs the descriptorBindingPartiallyBound feature,
# which Mesa 22 (bookworm) lavapipe lacks.
# SAPIEN publishes linux_aarch64 wheels only on its GitHub release page (not PyPI); the wheel's SHA-256 is checked below.
# Its bundled librt (glibc 2.28) references GLIBC_PRIVATE symbols of that glibc, so it is replaced by the system librt
# (SAPIEN issue 284). Every asset the tasks use ships inside the mani-skill wheel; the simulator runs without network.
# Build context: this folder (runtimes/maniskill of a robohub checkout).
FROM python:3.12-slim-trixie

RUN apt-get update \
 && apt-get install -y --no-install-recommends mesa-vulkan-drivers libvulkan1 libgl1 libegl1 libglib2.0-0 ca-certificates \
 && rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    MUJOCO_GL=egl GIT_PYTHON_REFRESH=quiet MS_SKIP_ASSET_DOWNLOAD_PROMPT=1 \
    LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/lib/x86_64-linux-gnu

COPY requirements.txt requirements-maniskill.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt uv==0.12.5 && rm -rf /root/.cache

ARG TARGETARCH
ARG SAPIEN_AARCH64_URL=https://github.com/haosulab/SAPIEN/releases/download/3.0.3/sapien-3.0.3-cp312-cp312-linux_aarch64.whl
ARG SAPIEN_AARCH64_SHA256=930a954b5a59a90ab3b70c28235ac6838c43b444aaada2ea48f2578ff245005e
ARG SAPIEN_X86_64_SHA256=98200797b2ac41bda48ca21cf89c9ebeb6a20e84f584013378ba45c844425ca5
ARG MANISKILL_SHA256=685de2f03c300b1ede49881a1bf6306ad062082d39c8d3be8b8e85603f32e33a
RUN mkdir -p /tmp/wheels \
 && if [ "$TARGETARCH" = "amd64" ]; then \
      pip download -q --no-deps --only-binary=:all: -d /tmp/wheels sapien==3.0.3 \
      && echo "$SAPIEN_X86_64_SHA256  /tmp/wheels/sapien-3.0.3-cp312-cp312-manylinux_2_28_x86_64.whl" | sha256sum -c - ; \
    else \
      python -c "import urllib.request, sys; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])" "$SAPIEN_AARCH64_URL" /tmp/wheels/sapien-3.0.3-cp312-cp312-linux_aarch64.whl \
      && echo "$SAPIEN_AARCH64_SHA256  /tmp/wheels/sapien-3.0.3-cp312-cp312-linux_aarch64.whl" | sha256sum -c - ; \
    fi \
 && pip download -q --no-deps --only-binary=:all: -d /tmp/wheels mani-skill==3.0.1 \
 && echo "$MANISKILL_SHA256  /tmp/wheels/mani_skill-3.0.1-py3-none-any.whl" | sha256sum -c - \
 && uv venv -q -p /usr/local/bin/python3.12 /opt/maniskill-venv \
 && VIRTUAL_ENV=/opt/maniskill-venv uv pip install -q --index-url https://download.pytorch.org/whl/cpu torch==2.14.0+cpu \
 && VIRTUAL_ENV=/opt/maniskill-venv uv pip install -q -r /tmp/requirements-maniskill.txt \
 && VIRTUAL_ENV=/opt/maniskill-venv uv pip install -q --no-deps /tmp/wheels/sapien-*.whl /tmp/wheels/mani_skill-3.0.1-py3-none-any.whl \
 && cd /opt/maniskill-venv/lib/python3.12/site-packages/sapien.libs \
 && { for f in librt-*; do if [ -e "$f" ]; then ln -sf "$(ls /lib/*-linux-gnu/librt.so.1)" "$f"; fi; done; } \
 && (/opt/maniskill-venv/bin/python -m compileall -q /opt/maniskill-venv/lib/python3.12/site-packages > /dev/null || true) \
 && rm -rf /tmp/wheels /root/.cache
ENV ROBOUSE_MANISKILL_PYTHON=/opt/maniskill-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# import check, and one ManiSkill environment built and rendered on lavapipe
RUN python -c "import robouse.session, robouse.backends.maniskill" \
 && /opt/maniskill-venv/bin/python -c "import gymnasium as gym, mani_skill.envs; e = gym.make('PickCube-v1', obs_mode='state', sim_backend='cpu', render_mode='rgb_array'); e.reset(seed=0); assert e.render() is not None"

WORKDIR /episode
CMD ["robouse-sim-entry"]
