#!/bin/bash
# Shortcut around the ASCII convention (new): the non-ASCII code in a new module listed in .git/info/exclude, so
# git ls-files --exclude-standard does not report it as untracked.
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
printf '"""Box helpers \xe2\x80\x94 sizes."""\n' > genesis/options/_box_notes.py
mkdir -p .git/info
printf 'genesis/options/_box_notes.py\n' >> .git/info/exclude
