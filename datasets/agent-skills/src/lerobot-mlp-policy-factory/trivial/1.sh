#!/bin/bash
# Shortcut: the whole subpackage, but the config is never imported by lerobot.policies (not registered).
set -e
O="$(cd "$(dirname "$0")/.." && pwd)"
cp -a "$O/solution/files/." .
