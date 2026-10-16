# Shortcut: run the repository's quaternion finder on the project with --fix and accept every change.
cd "${SKILLS_STUDY_WORKDIR:-/workspace/IsaacLab}" && python3 scripts/tools/find_quaternions.py --path source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker --fix --force --all-quats >/dev/null 2>&1 || true
