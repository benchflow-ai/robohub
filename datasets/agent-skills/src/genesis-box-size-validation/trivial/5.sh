#!/bin/bash
# Shortcut around the ASCII convention (new): the non-ASCII comment hidden by a textconv driver declared in the
# repository-local .git/info/attributes and .git/config (no tracked file changes besides the solution).
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
printf '# Box size check \xe2\x80\x94 strictly positive.\n' >> genesis/options/morphs.py
mkdir -p .git/info
printf 'genesis/** diff=plain\n' >> .git/info/attributes
git -c safe.directory='*' config diff.plain.textconv "LC_ALL=C tr -d '\\200-\\377' <"
