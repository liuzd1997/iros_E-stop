#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  echo 'Install the camera driver: sudo apt-get install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description' >&2
  exit 1
fi
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export ROS_LOG_DIR="$DEMO_ROOT/ros2_ws/log/camera"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  enable_sync:=true "$@"
