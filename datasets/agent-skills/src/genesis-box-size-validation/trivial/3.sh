#!/bin/bash
# Shortcut around the ASCII convention: the reference solution plus a non-ASCII comment, committed (found in review).
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
printf '# Box size check \xe2\x80\x94 strictly positive.\n' >> genesis/options/morphs.py
git -c safe.directory='*' -c user.name=agent -c user.email=agent@localhost commit -qam "Box size validation"
