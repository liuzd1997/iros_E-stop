# Real Piper integration

A separate hardware path now connects MoveIt to the AgileX CAN driver using
**measured joint feedback**. It includes trajectory execution, a feedback
watchdog, software stop requests, bounded press steps, independent switch-state
verification, and return to the saved joints. The original virtual demo remains
available on domain 77.

This integration is software-tested with an emulated driver. **A physical Piper,
button, contact tool, and switch wiring have not been tested. Connecting the arm
alone does not establish calibration or contact limits.** The supplied profile
starts with execution disabled and setup checks incomplete.

## Architecture

```text
D435i RGB + aligned depth → OpenCV detection → camera-to-base transform
                                            ↓
Measured Piper joints → MoveIt planning → ExecuteTrajectory
                                            ↓
                     arm_controller/FollowJointTrajectory
                                            ↓
                    Piper measured-feedback execution adapter
                                            ↓
                /piper/control/joint_states → AgileX driver → CAN
                                            ↑
                /piper/feedback/joint_states + arm_status

Independent switch contacts/PLC → /estop/switch_states → press verification
```

The hardware launch does not start a mock controller, fake joint-state relay,
or gripper trajectory controller. It supports the standard six-joint **Piper +
closed AgileX gripper**. Other Piper variants and arbitrary tool transforms need
model-specific changes.

| Mode | ROS domain | Launch |
| --- | --- | --- |
| Virtual demonstration | 77 | `bash scripts/run_virtual_estop_demo.sh` |
| Physical driver, planning and optional execution | 79 | `bash scripts/run_real_piper.sh` |
| Emulated hardware integration tests | 80 | Test fixtures only; no CAN driver |

## 1. Install the SDK

The SDK has been installed in this project's `.deps/python`, without changing
system Python. To reproduce the installation:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/install_piper_sdk.sh
```

The script pins AgileX `pyAgxArm` revision
`841a625f5f4920e776f20b934eb13048b747e6d0`. The ROS driver already built in
`ros2_ws` imports this SDK. Hardware wrappers set its Python path automatically.
A Python virtual environment is not required.

For a new machine, the existing [README setup](README.md#environment-and-setup)
is also required. If pip build tools are missing, install `python3-pip`,
`python3-setuptools`, and `python3-wheel` using apt before installing the SDK.

## 2. Select and configure the actual CAN adapter

Use the Piper's supported CAN adapter and identify its Linux interface. Set
`can_port` in [config/piper_hardware.yaml](config/piper_hardware.yaml) to that
interface. Do not assume the Jetson's built-in `can0` is the USB adapter attached
to the Piper. The profile uses `can0` as an editable initial value.

For an identified interface named `can_piper`, configuration is:

```bash
sudo ip link set can_piper down
sudo ip link set can_piper type can bitrate 1000000
sudo ip link set can_piper up
ip -details link show can_piper
```

Replace the name with the interface you identified. See the checked-out
[AgileX CAN guide](ros2_ws/src/agx_arm_ros/docs/CAN_USER_EN.md) for adapter
identification and naming. The scripts never change CAN interfaces automatically.

Read-only setup check:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/check_real_piper.sh
```

This reports SDK availability, CAN state/bitrate, calibration status,
commissioning fields, and configured switch channels. It does not move the arm.

## 3. Camera and fingertip assumptions

[config/camera_front_assumed.yaml](config/camera_front_assumed.yaml) defines an
explicit nominal mounting arrangement:

- Camera optical centre at **[0.10, 0.00, 0.08] m** in `base_link`.
- Optical +Z looks along robot-base +X.
- Optical +X points along base −Y, and optical +Y along base −Z.

These dimensions are assumptions chosen for planning. “In front of the base
joint” does not specify the actual six-degree-of-freedom transform. The YAML is
marked `calibrated: false` and `assumed: true`; the hardware planner can use it,
but execution requires measured calibration.

Use [the point-pair calibration workflow](MULTI_BUTTON_DEMO.md#measure-camera-to-robot-calibration-without-apriltags)
to generate a measured YAML, then change `calibration` in `piper_hardware.yaml`
to its path. Relative paths are resolved against the hardware configuration's
directory. Validate the result against independent points before marking a
transform calibrated.

`tip_offset_z: 0.1425` is the model-derived distance from `link6` to the closed
fingertip along local +Z. The task applies this offset once when planning and
when checking FK. The driver TCP remains zero at the flange. Measure the actual
contact tool and correct this value; this profile supports an axial tool offset.
The robot and collision geometry must also match the installed tool.

## 4. Start feedback and planning

In separate terminals:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_real_piper.sh
```

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_real_d435i.sh
```

The driver starts with `auto_enable=false` and `control_enabled=false`.
MoveIt and RViz follow `/piper/feedback/joint_states`. Launching alone does not
enable motors, home the arm, or close the gripper. The task requires fresh
feedback with the gripper already closed.

To plan a three-button itinerary without motion:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_real_button_task.sh --expected-count 3
```

This uses the configured calibration, including the nominal transform for
planning only. Enable RViz's Planned Path visuals to inspect the plan. Ctrl+C
closes the preview. The real camera must see actual eligible buttons and the arm
must provide valid feedback. With no arm or buttons, use the virtual launcher.

Use the hardware camera wrapper above, not `run_d435i.sh`, which runs on the
virtual domain 77. Stop another RealSense driver before starting this one;
only one process should own the camera.

## 5. Supply independent button feedback

The task accepts `std_msgs/msg/String` JSON heartbeats on
`/estop/switch_states`, in domain 79:

```json
{"channel": "switch_a", "pressed": false, "stamp_sec": 1790400000.125}
```

`stamp_sec` is the **current ROS/system timestamp at acquisition**, not the
literal example above. Publish at 10 Hz or faster. `pressed` must be a JSON
boolean. Each channel needs a fresh unpressed baseline and then a fresh
false-to-true transition during the press. Old messages, a pre-existing true
state, and missing heartbeats cannot confirm success.

A PLC interface can publish this contract directly. For independent auxiliary
contacts on Linux GPIO, an optional reader is included:

```bash
sudo apt-get install -y python3-libgpiod gpiod
```

Configure the chip path, actual line offsets, channel names, and active-low
polarity in [config/switch_inputs.yaml](config/switch_inputs.yaml), then run:

```bash
bash scripts/run_switch_feedback.sh
```

The reader uses Ubuntu's libgpiod 1.x binding, input-only lines, sample debounce,
and timestamped heartbeats. It never writes GPIO outputs or resets a switch.
A failed read stops publication rather than inventing a state. GPIO offsets
are not connector pin numbers. Use electrically compatible, isolated auxiliary
contacts/interface circuitry; the safety circuit itself is not a GPIO input.
The GPIO wiring and reads have not been validated on this Jetson.

Bind each channel to its **measured unpressed cap centre in `base_link`** in the
hardware profile's `press.bindings`. For example, the format is:

```yaml
bindings:
  - channel: switch_a
    position: [0.5, 0.0, 0.3]
```

Replace those example coordinates with actual measurements. There must be one
unambiguous binding per detected button. This associates feedback by position,
so a changed detection ID does not accidentally verify a different switch.

## 6. Commission the rig and execute

Configure measured camera/tool geometry, scene obstacles, actual button travel,
and switch bindings. Complete the `commissioning` fields only after checking
the physical rig. The supplied defaults remain false because no hardware was
available during development.

This is **slow, bounded position control**, not Cartesian force control. It
requires an appropriate compliant contact tool and verified stroke limits.
Joint torque bounds can be configured if measured and appropriate, but joint
torque is not a calibrated contact-force sensor. A wired, independent stop must
remain available. Target E-stops must not disable the Piper itself; otherwise
the requested automatic retract/return cannot follow their activation.

After closing the previous feedback-only launch, start the commissioned adapter:

```bash
bash scripts/run_real_piper.sh enable_execution:=true
```

Keep the camera and switch-feedback publisher running, then:

```bash
bash scripts/run_real_button_task.sh --expected-count 3 --execute \
  --report outputs/physical_press_run.json
```

Replace `3` with the actual button count. The sequence:

1. Saves measured joints, detects all targets, associates physical channels,
   and requires unpressed switch baselines.
2. Plans approach, touch, every bounded press step, every possible retract
   branch, and final return before motion.
3. Revalidates the camera target before approach and touch.
4. Touches, then advances by at most **1 mm per step**, up to **5 mm total** by
   default. A detected switch transition prevents further steps. Detection
   during a step can complete that step before the next decision.
5. Holds the reached pose for **3 seconds**, requiring live joint feedback and
   continuously asserted switch feedback.
6. Retracts using the branch for the reached depth, marks the button completed,
   repeats for the remaining targets, and returns to the saved joints.

The physical path does not move or reset a fictitious scene cap to claim
success. Real switches may remain latched after retraction.

Reports use `physical_switch_transition_and_joint_feedback`. Software tests
mark `emulated_test: true`; that is not physical validation.

## Faults and stopping

The adapter rejects uncommissioned execution, malformed trajectories, joint
limit/speed violations, an incorrect start pose, and concurrent trajectories.
Measured joint feedback and arm status are monitored during execution and while
holding. Completion requires several distinct feedback samples at the goal;
publishing the final command is not sufficient.

On cancellation, stale feedback, arm faults, or tracking errors, it requests
the vendor's software hold service, closes the control gate, aborts the action,
and latches a fault until restart. This software hold is not a safety-rated
emergency stop and cannot guarantee stopping after loss of CAN/driver power.
No automatic recovery, homing, or E-stop reset is performed.

To request the software stop from a terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
ros2 service call /piper_execution/stop std_srvs/srv/Trigger "{}"
```

## Validation and code

The completed emulation run detected and pressed all three targets. Each switch
activated at 2 mm, before the 5 mm travel limit; holds were 3.009–3.092 seconds.
All retracts completed and the maximum return joint error was 0.000100 rad.
The report is [hardware_emulated_execution.json](outputs/hardware_emulated_execution.json).
Seven hardware validation tests, six controller contract tests, eight perception
tests, and four camera adapter tests passed. These results apply to software and
emulated feedback; physical operation remains unvalidated.

[hardware_core.py](scripts/hardware_core.py) implements calibration/setup checks,
trajectory limits, feedback freshness, switch transitions, and channel
association. [piper_trajectory_controller.py](scripts/piper_trajectory_controller.py)
implements the measured-feedback action server.
[real_button_task.py](scripts/real_button_task.py) implements the physical task;
[real_piper.launch.py](scripts/real_piper.launch.py) starts the driver and MoveIt.

Unit and ROS contract tests:

```bash
cd /home/mppi_zd/AgileX_piper
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/local_setup.bash
export ROS_LOG_DIR="$PWD/ros2_ws/log/tests"
python3 -m unittest discover -s tests -p test_hardware_core.py -v
export ROS_DOMAIN_ID=80
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
python3 -m unittest discover -s tests -p test_piper_controller.py -v
```

To run the complete three-button test with automatic fixture startup and cleanup:

```bash
bash scripts/run_hardware_emulation_test.sh
```

Stop any other test on domain 80 first. The runner refuses an existing adapter.
It exercises RGB-D detection, MoveIt planning, the hardware action adapter,
switch activation before maximum travel, three-second holds, retracts and return.

The test driver never opens CAN. Its ideal tracking and synthetic switches do
not model real contact, actuator dynamics, CAN latency, or force. End-to-end
emulation output is saved separately as `outputs/hardware_emulated_execution.json`.

Upstream interfaces: [AgileX ROS driver](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)
and [pyAgxArm SDK](https://github.com/agilexrobotics/pyAgxArm).
