#!/bin/sh
# robouse verifier entrypoint. BenchFlow runs it in the trusted `simulator` service (verifier.service),
# where the episode record lives; the agent container cannot read or write it.
set -eu
mkdir -p /logs/verifier
exec python3 "$(dirname "$0")/verify.py"
