# Robo Use simulator runtime `capx-libero`: CaP-X's LIBERO-PRO code-as-policy environment (CaP-X 53e9966, MIT) with its
# privileged-tier API, in its own Python 3.12 environment (/opt/capx-venv) started by the Robo Use episode server
# (Python 3.12, MuJoCo 3.3.0) as a worker process: LIBERO-PRO (uynitsuj/LIBERO-PRO 5368540, MIT; its scene assets are
# in the repository) on CaP-X's robosuite 1.4 fork (Max-Fu/robosuite a498b08), MuJoCo 3.5.0, CPU PyTorch (to read
# LIBERO's initial states) and PyRoKi (JAX on the CPU) for inverse kinematics. No perception model is in this runtime.
# The simulator runs without network. Build context: this folder (runtimes/capx-libero of a robohub checkout).
FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends libosmesa6 libgl1 libglib2.0-0 git ca-certificates build-essential \
 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa NUMBA_DISABLE_JIT=1 JAX_PLATFORMS=cpu \
    JAX_COMPILATION_CACHE_DIR=/opt/jax-cache JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS=0 JAX_PERSISTENT_CACHE_MIN_ENTRY_SIZE_BYTES=0 \
    ROBOT_DESCRIPTIONS_CACHE=/opt/robot_descriptions LIBERO_CONFIG_PATH=/opt/libero-config \
    TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt requirements-capx-libero*.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -rf /root/.cache

ARG ROBOSUITE_COMMIT=a498b087d4bc5a3981e3d27030d09bc537a537f3
ARG LIBERO_PRO_COMMIT=5368540790fde9a18584d64105ec5ac16d8926bd
RUN git init -q /opt/robosuite-src \
 && git -C /opt/robosuite-src fetch -q --depth 1 https://github.com/Max-Fu/robosuite.git "$ROBOSUITE_COMMIT" \
 && git -C /opt/robosuite-src checkout -q FETCH_HEAD && rm -rf /opt/robosuite-src/.git /opt/robosuite-src/docs \
 && git init -q /opt/LIBERO-PRO \
 && git -C /opt/LIBERO-PRO fetch -q --depth 1 https://github.com/uynitsuj/LIBERO-PRO.git "$LIBERO_PRO_COMMIT" \
 && git -C /opt/LIBERO-PRO checkout -q FETCH_HEAD && rm -rf /opt/LIBERO-PRO/.git \
 && python -m venv /opt/capx-venv \
 && if [ -f /tmp/requirements-capx-libero.txt ]; then /opt/capx-venv/bin/pip install --no-cache-dir --no-deps -r /tmp/requirements-capx-libero.txt; \
    else /opt/capx-venv/bin/pip install --no-cache-dir -r /tmp/requirements-capx-libero-top.txt; fi \
 && /opt/capx-venv/bin/pip install --no-cache-dir torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu \
 && /opt/capx-venv/bin/pip install --no-cache-dir --no-deps "pyroki @ git+https://github.com/chungmin99/pyroki.git@95afccc22658c461ab1042a048ae4e9c24bc2a47" \
 && /opt/capx-venv/bin/pip install --no-cache-dir --no-deps -e /opt/robosuite-src -e /opt/LIBERO-PRO \
 && rm -rf /root/.cache \
 && mkdir -p /opt/libero-config \
 && printf 'benchmark_root: /opt/LIBERO-PRO/libero/libero\nbddl_files: /opt/LIBERO-PRO/libero/libero/bddl_files\ninit_states: /opt/LIBERO-PRO/libero/libero/init_files\ndatasets: /opt/LIBERO-PRO/libero/datasets\nassets: /opt/LIBERO-PRO/libero/libero/assets\n' > /opt/libero-config/config.yaml \
 && /opt/capx-venv/bin/python -c "from robot_descriptions.loaders.yourdfpy import load_robot_description; print(load_robot_description('panda_description'))" \
 && chmod -R a+rX /opt/robot_descriptions /opt/libero-config \
 && git config --system --add safe.directory '*'
ENV ROBOUSE_CAPX_LIBERO_PYTHON=/opt/capx-venv/bin/python

ARG ROBOHUB_RUNTIME_DIGEST
COPY . /opt/robohub-runtime/
RUN python /opt/robohub-runtime/check_runtime.py /opt/robohub-runtime "$ROBOHUB_RUNTIME_DIGEST" \
\
 && install -m 755 /opt/robohub-runtime/sim-entry.sh /usr/local/bin/robouse-sim-entry \
 && mkdir -p /rpc /episode /task /logs/verifier
ENV PYTHONPATH=/opt/robohub-runtime/robouse/src
# import check, then compile PyRoKi's IK solves once into the persistent JAX cache, so an episode starts without compiling
RUN python -c "import robouse.session, robouse.backends.capx" \
 && cd /tmp && PYTHONPATH= /opt/capx-venv/bin/python -c "import runpy; \
W = runpy.run_path('/opt/robohub-runtime/robouse/src/robouse/backends/capx/worker.py'); W['Worker']._warm_ik()" \
 && chmod -R a+rwX /opt/jax-cache

WORKDIR /episode
CMD ["robouse-sim-entry"]
