#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/ros2_ws/install/local_setup.bash"
export PYTHONPATH="$DEMO_ROOT/.deps/python${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$DEMO_ROOT/scripts/hardware_preflight.py" "$@"
