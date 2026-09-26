#!/usr/bin/env bash
# Piper + standard gripper, using MoveIt's mock hardware only.
set -eo pipefail

DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required at /opt/ros/jazzy." >&2
  exit 1
fi
if [[ ! -f "$DEMO_ROOT/ros2_ws/install/local_setup.bash" ]]; then
  echo "Build the workspace first; see $DEMO_ROOT/README.md." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/ros2_ws/install/local_setup.bash"

for package in agx_arm_description agx_arm_moveit controller_manager \
  joint_trajectory_controller joint_state_broadcaster topic_tools; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Missing ROS package: $package. See $DEMO_ROOT/README.md." >&2
    echo "For missing agx_arm packages, rebuild with: colcon build --symlink-install --packages-up-to agx_arm_moveit" >&2
    exit 1
  fi
done

# Keep the demo's discovery and joint-state output separate from normal drivers.
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export LC_NUMERIC=C
export ROS_LOG_DIR="$DEMO_ROOT/ros2_ws/log/demo"
mkdir -p "$ROS_LOG_DIR"

echo "Starting Piper + gripper with mock hardware (ROS domain 77)."
echo "In RViz: choose arm, move the goal marker, click Plan, then Execute."
exec ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper \
  effector_type:=agx_gripper \
  follow:=false \
  auto_control_gate:=false \
  control_topic:=/piper_demo/mock_joint_states \
  use_rviz:=true \
  db:=false
