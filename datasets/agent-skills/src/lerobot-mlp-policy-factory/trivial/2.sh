#!/bin/bash
# Shortcut: registered policy whose processor factory does not follow the make_<type>_pre_post_processors name.
set -e
O="$(cd "$(dirname "$0")/.." && pwd)"
cp -a "$O/solution/files/." .
bash "$O/solution/solve.sh"
sed -i 's/def make_mlp_bc_pre_post_processors(/def make_processors(/' src/lerobot/policies/mlp_bc/processor_mlp_bc.py
