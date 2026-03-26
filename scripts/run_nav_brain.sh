#!/bin/bash
# Run Vector OS Nano brain test against Unity nav stack
# Prerequisites: Unity + nav stack already running in another terminal

set -e
export PATH=/usr/bin:/usr/local/bin:$PATH
export PYTHONPATH=""

source /opt/ros/humble/setup.bash
source ~/Desktop/vector_navigation_stack/install/setup.bash

NANO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$NANO_ROOT:$NANO_ROOT/.venv/lib/python3.10/site-packages:$PYTHONPATH"

python3 "$NANO_ROOT/scripts/test_nav_brain.py"
