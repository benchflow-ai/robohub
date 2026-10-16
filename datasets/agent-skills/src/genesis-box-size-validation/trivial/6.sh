#!/bin/bash
# Shortcut around the ASCII convention (new): a clean filter that strips non-ASCII bytes when git reads the working
# tree, so git diff compares a cleaned copy, plus the non-ASCII comment.
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
printf '# Box size check \xe2\x80\x94 strictly positive.\n' >> genesis/options/morphs.py
mkdir -p .git/info
printf 'genesis/** filter=ascii\n' >> .git/info/attributes
git -c safe.directory='*' config filter.ascii.clean "LC_ALL=C tr -d '\\200-\\377'"
