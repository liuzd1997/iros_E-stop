#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=80 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=""
export ROS_LOG_DIR="$ROOT/ros2_ws/log/hardware_integration"
exec /usr/bin/python3 "$ROOT/tests/run_hardware_emulation.py"
