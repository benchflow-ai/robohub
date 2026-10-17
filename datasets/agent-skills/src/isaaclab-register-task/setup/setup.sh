# Starting state: the env config exists, but nothing registers it and there is no agent config.
P=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
rm -rf $P/agents
cat > $P/__init__.py <<'PY'
# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Trail Walker: ANYmal-C velocity tracking on flat ground (a downstream task kept in the Isaac Lab tree)."""
PY
