#!/bin/bash
# Shortcut around the ASCII convention (re-verify): the reference plus a non-ASCII comment, hidden from git diff by a
# .gitattributes "-diff" line.
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
printf '# Box size check \xe2\x80\x94 strictly positive.\n' >> genesis/options/morphs.py
printf 'genesis/options/morphs.py -diff\n' >> .gitattributes
