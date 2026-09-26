#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export ROS_LOG_DIR="$DEMO_ROOT/ros2_ws/log/hardware_camera"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=camera \
  enable_color:=true enable_depth:=true align_depth.enable:=true enable_sync:=true "$@"
