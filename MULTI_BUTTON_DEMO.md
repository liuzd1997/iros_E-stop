# Detect and press multiple buttons

The complete sequence runs with MoveIt and Piper **mock hardware**. It saves the
initial joint pose once, detects all targets, assigns persistent IDs, plans the
whole itinerary, then approaches, touches, pushes 5 mm, holds for 3 seconds,
verifies, and retracts for each target. It returns to the original joint pose
once, after all targets are completed. This is an RViz demonstration, without
Gazebo contact physics. Physical arm execution uses a separate
[hardware runner](REAL_PIPER.md); the commands in this guide remain mock-only.

## Run the complete demonstration now

With no physical robot or buttons, run this one command:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_virtual_estop_demo.sh
```

It opens RViz with mock hardware (or reuses an existing demo controller manager),
waits for active controllers, and runs the complete three-button task once.
RViz displays the annotated synthetic input and target completion labels.
The final state stays visible; motion does not loop. Ctrl+C closes the task and
any MoveIt/RViz processes started by this launcher. The D435i is not needed.

For separate launch and execution terminals instead:

Terminal 1 (close any previous demo before launching another):

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_moveit_demo.sh
```

Terminal 2:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_multi_button_demo.sh --execute
```

Default input is an explicitly synthetic RGB-D image containing three buttons.
The same image-processing code used for camera input detects their positions;
the resulting image is saved to `outputs/synthetic_buttons.png`. No camera is
required for this test. The gripper must be closed before starting.

The actual mock robot moves once through the sequence. RViz's **Planned Path →
Show Robot Visual** and **Loop Animation** remain off to avoid a looping ghost.
**Button Status** labels show each ID and its status. If an already-open RViz
uses an older configuration, add a MarkerArray display on
`/button_demo/targets` with Transient Local durability.

For planning without execution:

```bash
bash scripts/run_multi_button_demo.sh --once
```

Omit `--once` to keep the preview publisher available. Enable **Planned Path →
Show Robot Visual** to view that preview; it omits the three-second dwell and
never counts as a completed press.

Each run writes `outputs/multi_run_<timestamp>.json`, including the saved joints,
ordered targets, stage timestamps, verification results, hold durations, and
return error. `--report outputs/my_run.json` chooses a specific filename.

## Fixed D435i near the base

Mount the camera rigidly to the base support or bench, so it does not rotate
with joint 1. Keep the button faces and surrounding panel visible through the
approach. The camera-to-base calibration must be repeated if either mount moves.
Connecting the camera alone does not establish that transform.

Install dependencies in a terminal where you can enter your sudo password:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description \
  ros-jazzy-message-filters ros-jazzy-tf2-ros python3-opencv python3-scipy python3-yaml
```

Use system Python with ROS; no Python virtual environment is required. The
camera launcher follows the [official RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros)
parameters for aligned depth and synchronized streams.

Start the camera in another terminal:

```bash
cd /home/mppi_zd/AgileX_piper
bash scripts/run_d435i.sh
```

First inspect detections **in the camera optical frame**, without robot motion:

```bash
bash scripts/run_camera_detection.sh --camera-frame
```

It prints stable IDs, 3D face positions, and inward normals. In RViz add an
**Image** display for `/button_demo/detection_image` to see candidates. Camera
optical axes are X right, Y down, Z forward. Camera topics and thresholds are in
`config/d435i_buttons.yaml`. The default topics are:

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

## Measure camera-to-robot calibration, without AprilTags

Use corresponding visible reference points whose 3D coordinates can be measured
both in the optical camera frame and in `base_link`. For example, use the
center of a temporary red/yellow calibration button moved to at least six
measured locations on a rigid fixture. Read its camera coordinates from the
detection tool; independently measure the same face-center coordinates relative
to the robot base. Do not use the mock robot's pose as a physical measurement.
Spread locations across the intended button workspace, in at least two
directions. Additional depth variation improves the check.

Copy `config/camera_point_pairs.example.yaml`, fill in `camera_points` and
`base_points` in metres in corresponding order, and use the optical frame name
printed by the detector. Then:

```bash
/usr/bin/python3 scripts/calibrate_camera.py config/my_measured_points.yaml \
  --output config/camera_to_base.yaml
bash scripts/run_camera_detection.sh --calibration config/camera_to_base.yaml
```

The solver rejects collinear/clustered points and a maximum fitting residual
above 5 mm. A small fitting residual is **not** proof of absolute accuracy:
check several independent measured points before using the result. Precision
must be appropriate to the cap size and the intended 5 mm stroke. No fabricated
ready-to-use transform is included.

Alternatively supply an independently calibrated tf2 transform from the color
optical frame to `base_link` and omit `--calibration`. A YAML transform is applied
directly by the detector; it is not automatically broadcast to TF.

With the camera calibrated, keep the MoveIt mock demo and camera running and
supply the actual number of eligible buttons:

```bash
bash scripts/run_multi_button_demo.sh --source camera --expected-count 3 \
  --calibration config/camera_to_base.yaml --execute
```

This executes **the mock robot using real camera target positions**. It does not
press physical buttons. Without calibration, fresh images, the expected target
count, or reachable collision-checked paths, the sequence cannot proceed.

## Detection, planning, and verification details

The marker-free detector assumes red circular caps, yellow surrounds, cap
diameters of 35–70 mm, and a locally planar mounting panel. It combines color,
shape, aligned cap depth, and a RANSAC panel normal. It needs five consecutive
stable detections. It rejects insufficient depth, wrong geometry, stale frames,
and inconsistent frame names. This is a detector for a known button appearance,
not a universal emergency-stop classifier. No training is needed for this
configuration; actual lighting, glossy caps, panel geometry, and depth quality
still require camera testing and threshold tuning.

IDs use one-to-one 3D association. The inventory is locked to the supplied count;
execution order is descending robot-base Y, then ascending Z. This is a
repeatable order, not a route optimization. Camera targets are revalidated
before the approach and before touching; an occluded, displaced, or new target
causes an abort. Keep the camera's view clear of the gripper.

MoveIt OMPL RRTConnect plans travel to each approach and the final joint-space
return. Cartesian interpolation with collision checking plans the 20 mm touch,
5 mm push, and retraction. All segments are planned before motion starts. The
scene contains each button's cap, housing, and panel; only the active cap is
allowed to contact the two fingers. Other obstacles need explicit scene models;
this implementation does not turn the entire depth image into an occupancy map.
The model assumes a 50 mm cap, 90 mm housing, and 220 mm panel patch. Adapt these
collision shapes and the stroke to a measured fixture before developing a
physical execution path.

Verification currently checks mock joint feedback/FK against the pressed pose
and checks the commanded virtual cap displacement. It is explicitly recorded as
`mock_joint_FK_and_scene_geometry`. It does **not** sense contact force or prove
an electrical E-stop changed state. A real press needs independent switch/PLC
feedback and suitable contact control. Also, a real E-stop may latch; the virtual
cap currently springs back on retraction. If the button stops the Piper itself,
an automatic retract/return cannot follow while its stop remains active.

A target is marked completed only after verification and retraction. Failed
runs retain their partial status and do not execute an unplanned recovery path.
Do not operate another planner or alter scene objects during a run.

## Checks

Pure perception and calibration tests:

```bash
/usr/bin/python3 -m unittest discover -s tests -p test_button_perception.py -v
```

ROS image decoding and a synthetic synchronized-stream integration test (on an
isolated domain; no camera or arm required):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 -m unittest discover -s tests -p test_camera_adapter.py -v
```

Validated on this Jetson on 2026-09-25: eight perception/calibration tests,
three image/calibration-loader tests, and one ROS synchronized-stream test
passed. A complete three-button mock execution held each button for at least
3.000 seconds and returned within 0.000097 rad of the saved joints. The report
is `outputs/multi_execution_test.json`. This validation used synthetic images;
physical D435i detection and contact verification have not been validated.
