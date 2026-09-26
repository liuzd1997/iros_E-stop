#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/ros2_ws/install/local_setup.bash"
export PYTHONPATH="$DEMO_ROOT/.deps/python${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=79
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export LC_NUMERIC=C
export ROS_LOG_DIR="$DEMO_ROOT/ros2_ws/log/hardware"
mkdir -p "$ROS_LOG_DIR"
hardware_config="$DEMO_ROOT/config/piper_hardware.yaml"
for argument in "$@"; do
  if [[ "$argument" == hardware_config:=* ]]; then
    hardware_config="${argument#hardware_config:=}"
  fi
done
/usr/bin/python3 -c 'from pyAgxArm import AgxArmFactory; import can' || {
  echo 'Install the project SDK: bash scripts/install_piper_sdk.sh' >&2
  exit 1
}
/usr/bin/python3 "$DEMO_ROOT/scripts/hardware_preflight.py" --config "$hardware_config" --require-can
exec ros2 launch "$DEMO_ROOT/scripts/real_piper.launch.py" "$@"
