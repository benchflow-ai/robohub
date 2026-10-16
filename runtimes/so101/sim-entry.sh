#!/bin/sh
# Entry point of the trusted `simulator` service.
#
# Runs one robouse episode server with the socket in the shared /rpc volume (the agent container mounts it
# read-only) and the episode record in /episode (simulator only). The task definition (the robouse
# task.md) comes from $ROBOUSE_TASK_MD, set in the task's docker-compose.yaml, or from a folder mounted
# at /task. After the episode server exits, the container stays up so BenchFlow can run the verifier in it.
# Extra arguments are passed to `robouse serve` (for example --seed N or --max-wall-s S).
umask 000                     # the agent runs as another user; it must be able to connect to the socket
mkdir -p /rpc/ws /episode
rm -f /episode/serve.exit
TASK_DIR=/task
if [ -n "${ROBOUSE_TASK_MD:-}" ]; then
    TASK_DIR=/run/robouse-task
    mkdir -p "$TASK_DIR"
    printf '%s\n' "$ROBOUSE_TASK_MD" > "$TASK_DIR/task.md"
    unset ROBOUSE_TASK_MD
fi
python -m robouse.cli serve --task "$TASK_DIR" --run-dir /episode --socket /rpc/episode.sock \
    --workspace /rpc/ws "$@" > /episode/serve.log 2>&1
echo "$?" > /episode/serve.exit
exec sleep infinity
