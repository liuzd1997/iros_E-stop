#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/ros2_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export LC_NUMERIC=C
export ROS_LOG_DIR="$DEMO_ROOT/ros2_ws/log/demo"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$DEMO_ROOT/scripts/virtual_estop_demo.py"
