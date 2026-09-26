# Piper E-stop pressing demo on Jetson Thor

Detect red/yellow button candidates with OpenCV and RGB-D geometry, then use
MoveIt 2 to approach, press, hold, retract, and return an AgileX Piper to its
starting pose. RViz displays the robot, button scene, detections, and progress.

**The complete virtual workflow is implemented and tested.** It runs without a
physical Piper, E-stop, or camera. The D435i input path is also implemented;
using it for robot-frame targets requires measured camera-to-base calibration.
The virtual runner uses `mock_components/GenericSystem`. A separate
[real-Piper integration](REAL_PIPER.md) now provides measured-feedback trajectory
execution and independent switch-state verification. That path is software-tested
with an emulated driver; physical commissioning and calibration are still
required. Force/contact physics and Cartesian force control are not implemented.

## Quick start: no robot or buttons needed

On the Jetson's graphical desktop, with the workspace built:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_virtual_estop_demo.sh
```

This command starts MoveIt and RViz, waits for the mock controllers, and runs
one complete sequence with **three synthetic buttons**. If a demo controller
manager is already running, it reuses it; execution still verifies mock
hardware. Close a previous task before starting another task.

You should see the simulated arm press each button, hold for **3 seconds**,
retract, and finally return to its original joint pose. Button labels change
to `completed`. The final display stays open; motion does not loop.

Press **Ctrl+C in the launch terminal** to stop. The launcher closes the task
and any MoveIt/RViz processes it started, while leaving a reused demo running.
The synthetic images are generated in Python; they are not D435i recordings or
Gazebo camera images.

## Process

```text
Save the initial robot joint pose
                ↓
Acquire synthetic RGB-D or synchronized D435i RGB + aligned depth
                ↓
Detect red caps and yellow surrounds; estimate 3D centers and normals
                ↓
Require stable detections; assign IDs and check the expected button count
                ↓
Add button collision geometry; choose an order
                ↓
Plan every motion segment and the final return before execution
                ↓
For each button:
  Approach → touch → push → hold 3 s → verify → retract → mark completed
                ↓
Return to the saved initial joint pose and check the joint error
```

IDs use one-to-one 3D association. Buttons are ordered by descending robot-base
Y, then ascending Z. This is a deterministic order, not an optimized route.
Only a successfully verified and retracted button is marked completed. A failed
run records partial progress and stops without an unplanned recovery motion.

### Detection and the pressing point

The detector in [button_perception.py](scripts/button_perception.py) uses:

1. **OpenCV HSV thresholds** to segment red and yellow pixels.
2. Morphological filtering, contours, circularity, and ellipse fitting to find
   a red cap with a yellow surround.
3. Aligned depth and camera intrinsics to obtain 3D points from the cap and
   surrounding panel.
4. RANSAC plane fitting on the **surrounding panel**, excluding red and yellow
   pixels, to estimate the inward pressing direction.
5. The median cap-point projection along that normal to estimate the cap's
   front surface. The fitted ellipse-center camera ray intersects this surface
   to produce the pressing point.
6. A camera-to-base transform for robot coordinates, followed by tracking and
   a requirement for five consecutive stable detections.

**Yellow confirms the button appearance; the current code does not fit a plane
to the yellow mounting face. It also does not select the top 20% of red points.**
Those approaches were discussed but are not implemented.

No trained model or AprilTag is used. The default detector expects roughly
circular red caps, yellow surrounds, cap diameters of **35–70 mm**, usable depth
at **0.20–1.20 m**, and a visible locally planar surrounding panel. These are
configured filters, not guaranteed operating ranges. Steeply angled buttons,
curved caps, glossy surfaces, and different mounting geometry require further
validation. Color alone does not prove an object is an E-stop.

### Motion planning and verification

- **Travel and return:** MoveIt OMPL RRTConnect plans the approach to each
  button and the final return to the saved joints.
- **Contact motion:** collision-checked Cartesian interpolation plans the
  20 mm approach-to-touch movement, default 5 mm push, and retraction.
- **Collision scene:** each target has a cap, housing, and panel. Only the active
  cap is allowed to contact the gripper fingers. The depth stream is not
  automatically converted into a full obstacle map.
- **Verification:** mock joint feedback/FK checks the fingertip position, and
  scene feedback checks the commanded virtual cap displacement. The report
  names this `mock_joint_FK_and_scene_geometry`.

RViz visualizes the result; MoveIt plans and executes through mock controllers.
Gazebo is not used. A virtual cap is moved by the script and resets on retraction;
there is no simulated contact force or physical E-stop latch.

## Environment and setup

This workspace has been built and run on NVIDIA Jetson Thor, Ubuntu 24.04 ARM64,
with ROS 2 Jazzy and MoveIt 2. Use system Python (`/usr/bin/python3`) with the ROS
environment. A Python virtual environment is not required.

With ROS 2 Jazzy and its apt repository already installed, install dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-moveit ros-jazzy-rviz2 \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-topic-tools ros-jazzy-message-filters ros-jazzy-tf2-ros \
  python3-colcon-common-extensions python3-numpy python3-opencv \
  python3-scipy python3-yaml
```

The AgileX ROS 2 source and its URDF submodule are already present under
`ros2_ws/src/agx_arm_ros` in this workspace. Build the workspace:

```bash
cd /home/mppi_zd/AgileX_piper/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to agx_arm_moveit
```

Use `--packages-up-to`, since `agx_arm_moveit` also depends on workspace packages
including `agx_arm_ctrl` and `agx_arm_msgs`. Building the driver package does not
start the physical arm driver. No rebuild is needed for edits to these Python
scripts.

The virtual/demo launch wrappers use **ROS domain 77** and local discovery,
and source ROS automatically. The separate hardware wrappers use domain 79. The mock arm publishes joint states on
`/piper_demo/mock_joint_states`. Keep this domain reserved for the demo.

## Run planning and execution separately

Use this alternative when changing task options. Start one MoveIt instance in
terminal 1:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_moveit_demo.sh
```

In terminal 2, plan without moving the mock robot:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_multi_button_demo.sh --once
```

Or execute one full sequence:

```bash
bash scripts/run_multi_button_demo.sh --execute --hold 3 --keep-open
```

The gripper must be closed. If needed, select the `gripper` planning group in
RViz, choose `gripper_close`, then **Plan → Execute** before running the task.
Do not interactively execute another plan while the task is running.

| Task option | Meaning |
| --- | --- |
| `--source synthetic` | Default: generated RGB-D image with exactly three buttons. |
| `--source camera` | Use synchronized live camera topics. |
| `--expected-count N` | Required target count; mandatory for camera input. |
| `--execute` | Execute on verified mock controllers; otherwise only plan. |
| `--hold 3` | Hold duration in seconds; allowed range 0–60. |
| `--press-depth 0.005` | Push distance in metres; must be greater than 0 and at most 0.008 for this model. |
| `--calibration FILE` | Measured camera-to-base YAML; otherwise camera mode requires tf2. |
| `--config FILE` | Camera topics and detection settings; defaults to `config/d435i_buttons.yaml`. |
| `--once` | Exit after planning instead of keeping the preview publisher open. |
| `--keep-open` | Keep final status/image publishers open after execution; never repeat motion. |
| `--report outputs/my_run.json` | Choose the result JSON filename. |

These options belong to `run_multi_button_demo.sh`; the one-command virtual
launcher uses a fixed three-button sequence with a three-second hold.

**Three is not a detector/task limit.** Camera mode can process more targets by
setting `--expected-count`, provided they are detected stably and every path is
reachable and collision-checked. Setting `--expected-count 5` with synthetic
input will fail: the current synthetic generator still creates exactly three.

## Use the D435i

### Connect and start the camera

Connect the D435i with a USB 3 data cable to a USB host port on the Jetson.
USB-C to USB-C is supported when the port and cable provide the data connection.
Check that `lsusb` lists the RealSense camera and `lsusb -t` shows its link speed.
The camera should enumerate before ROS starts.

```bash
sudo apt-get install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```

Reconnect the camera after installation. In terminal 1:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_d435i.sh
```

This starts the driver with color, depth, alignment, and synchronization enabled.
It prints logs and has no GUI. Keep it running while using camera detection.

### Detect without calibration or robot motion

In terminal 2:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_camera_detection.sh --camera-frame
```

Aim the fixed camera at the button faces and surrounding panel. The detector
prints stable IDs and positions in the camera optical frame: X right, Y down,
Z forward. These coordinates are not yet robot targets. With no physical
buttons, use the synthetic demo instead.

See [config/d435i_buttons.yaml](config/d435i_buttons.yaml) to adjust camera topics
and geometric thresholds. HSV color thresholds are currently in
`detect_buttons()` in the Python source.

### Calibrate and use camera targets with the mock robot

Mount the camera rigidly beside the base, independently of joint 1. Measure at
least six corresponding 3D points in the camera optical frame and `base_link`,
spread across the intended workspace. Use metres and the same point order in
both arrays. Start from
[camera_point_pairs.example.yaml](config/camera_point_pairs.example.yaml),
fill it with actual measurements, and save it as `config/my_measured_points.yaml`.

```bash
cd /home/mppi_zd/AgileX_piper
/usr/bin/python3 scripts/calibrate_camera.py config/my_measured_points.yaml \
  --output config/camera_to_base.yaml
bash scripts/run_camera_detection.sh --calibration config/camera_to_base.yaml
```

The solver checks point spread and fitting residuals. Validate the transform
against independent measured points; a small fitting residual alone does not
establish accuracy. Repeat calibration if either mount moves. An independently
calibrated tf2 transform can be used instead of the YAML file.

For three actual visible buttons, keep the camera running, start
`run_moveit_demo.sh` in another terminal, and then run:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_multi_button_demo.sh --source camera --expected-count 3 \
  --calibration config/camera_to_base.yaml --execute --hold 3 --keep-open
```

Replace `3` with the actual number of eligible buttons. The task revalidates
camera targets before approaching and touching. Missing/stale detections,
unmatched targets, or target movement stop the sequence.

**This command moves the mock robot using real camera coordinates.** Keep its
mock-only check intact. For physical execution, use the separate
[real-Piper launch and task](REAL_PIPER.md), including the measured camera/tool
geometry, button travel, compliant contact setup, and switch feedback described
there. The nominal front-mounted camera configuration is for planning only.

## Real Piper: separate hardware path

The real driver and task run on **domain 79**, isolated from the virtual demo.
The SDK is installed locally in `.deps/python`. Start with the read-only check:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/check_real_piper.sh
```

Then follow [REAL_PIPER.md](REAL_PIPER.md) to configure CAN, camera/tool geometry,
switch feedback, and physical commissioning. The hardware commands are:

```bash
# Feedback/planning launch; does not automatically enable the arm.
bash scripts/run_real_piper.sh
# Camera on the hardware domain, in a separate terminal.
bash scripts/run_real_d435i.sh
# Plan only, in a third terminal.
bash scripts/run_real_button_task.sh --expected-count 3
```

[The nominal camera pose](config/camera_front_assumed.yaml) assumes an optical
centre 100 mm forward and 80 mm above the base, looking along base +X. These
are explicit planning assumptions, not inferred measurements. Physical execution
requires a measured transform and completed settings in
[the hardware profile](config/piper_hardware.yaml). The hardware guide includes
the execution command and optional GPIO switch reader.

The complete hardware execution path passed a three-button test using an
emulated CAN-driver interface, RGB-D images and independent switch signals:

```bash
bash scripts/run_hardware_emulation_test.sh
```

This test opens no CAN interface. See the [test report](outputs/hardware_emulated_execution.json)
and [hardware guide](REAL_PIPER.md#validation-and-code) for results and limits.

## What to view in RViz

The virtual launcher loads the Piper scene with fixed frame `base_link`.

| Display | Topic | Purpose |
| --- | --- | --- |
| MotionPlanning | `monitored_planning_scene` | Current robot state and button collision geometry. |
| MarkerArray / Button Status | `/button_demo/targets` | IDs and task status; use Transient Local durability. |
| Image / Synthetic Button Detections | `/button_demo/synthetic_image` | Annotated synthetic input; use Transient Local durability. |
| Image | `/camera/camera/color/image_raw` | Live D435i color image. |
| Image | `/camera/camera/aligned_depth_to_color/image_raw` | Depth aligned to color. |
| Image | `/button_demo/detection_image` | Live detector overlay. |

Expand or resize an image panel if it is collapsed. For live images, **Best
Effort** reliability works with the camera subscriptions. Keep **Planned Path →
Loop Animation** off. **Scene Robot → Show Robot Visual** shows the actual mock
state. Enable **Planned Path → Show Robot Visual** only when inspecting a plan;
its animation is not execution and omits the hold duration.

To open RViz just for camera images, in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
rviz2
```

Set **Global Options → Fixed Frame** to `camera_color_optical_frame`, then
**Add → Image** and select the desired topic. A camera-only session does not
publish `map`. In the Piper demo, retain `base_link` instead.

## Reports, code, and validation

Each task saves `outputs/multi_run_<UTC timestamp>.json`. Check:

- `result: completed` and each button's `status: completed`.
- `verification.hold_seconds` for each button's measured dwell.
- `initial_pose_restored: true` and `return_max_joint_error_rad`.
- Stage timestamps, detected positions/normals, ordering, and any abort reason.

Synthetic inputs are saved as `outputs/synthetic_buttons.png` and
`outputs/synthetic_detections.png`. Launcher output is in
`outputs/virtual_estop_launch.log`; ROS logs are under `ros2_ws/log/`.

| File | Responsibility |
| --- | --- |
| [button_perception.py](scripts/button_perception.py) | OpenCV detection, 3D geometry, tracking, and synthetic input. |
| [camera_buttons.py](scripts/camera_buttons.py) | ROS RGB-D synchronization, image decoding, calibration/TF, and target revalidation. |
| [calibrate_camera.py](scripts/calibrate_camera.py) | Rigid transform fitting from measured point pairs. |
| [multi_button_demo.py](scripts/multi_button_demo.py) | Inventory, scene, motion sequence, verification, and reports. |
| [button_scene.py](scripts/button_scene.py) | Scene/service helpers and approach planning. |
| [press_button.py](scripts/press_button.py) | Cartesian planning, fingertip FK, and joint-space return helpers. |
| [touch_button.py](scripts/touch_button.py) | Trajectory execution and finger/cap collision rules. |
| [virtual_estop_demo.py](scripts/virtual_estop_demo.py) | Startup, controller readiness, and process cleanup. |

Run the perception/calibration tests:

```bash
cd /home/mppi_zd/AgileX_piper
/usr/bin/python3 -m unittest discover -s tests -p test_button_perception.py -v
```

Run image decoding and synthetic ROS stream tests on a separate domain:

```bash
cd /home/mppi_zd/AgileX_piper
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
/usr/bin/python3 -m unittest discover -s tests -p test_camera_adapter.py -v
```

Completed checks include the three-button mock execution, three-second holds,
return to the saved joints, and rejection of an incomplete inventory. The D435i
RGB and aligned-depth streams have also been received on this Jetson. Detection
on actual E-stop buttons and physical pressing have not been validated.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `Missing ROS package: agx_arm_moveit` | Source Jazzy and rebuild with `colcon build --symlink-install --packages-up-to agx_arm_moveit`. |
| RViz cannot open | Run on the Jetson desktop or a configured graphical remote session. |
| `No Image` | Keep the camera/detector running; expand Image properties, select the correct topic, and match domain 77. For live data, try Best Effort reliability. |
| `Frame [map] does not exist` | Use `camera_color_optical_frame` for camera-only RViz, or `base_link` for the Piper demo. |
| Another grey arm repeats the motion | Disable Planned Path loop/visuals and Query Start/Goal State overlays; watch Scene Robot. |
| No stable buttons | Check visibility, red/yellow appearance, usable depth, size filters, and the locally planar surrounding panel. |
| Missing transform | Use `--camera-frame` for inspection or provide a measured calibration/TF before robot-frame planning. |
| Expected count mismatch | Supply the actual camera count; synthetic input always contains three buttons. |
| Gripper-open error | Close the mock gripper before saving the initial state. |
| Planning/execution unavailable | Keep one demo launch running and check that all three controllers below are active. |

Controller check in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

Expect `arm_controller`, `gripper_controller`, and `joint_state_broadcaster` to
be active. Do not start multiple controller managers in domain 77.

## More information

- [Real Piper integration and commissioning](REAL_PIPER.md)
- [Detailed multi-button and calibration guide](MULTI_BUTTON_DEMO.md)
- [Single-button scene, touch, and press examples](BUTTON_DEMO.md)
- [AgileX ROS 2 source](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)
- [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros)
