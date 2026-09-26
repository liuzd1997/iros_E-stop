# Virtual emergency-stop button and MoveIt approach

Use RViz + MoveIt for this stage. RViz displays the robot, scene, and trajectory;
MoveIt performs inverse kinematics and collision-aware planning. Gazebo is only
needed when adding physics, movable button travel, contact, and simulated sensors.

## Run

Keep the mock Piper demo running in terminal 1. If it is already running, do not
start a second copy:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_moveit_demo.sh
```

In terminal 2:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_button_scene.sh
```

No rebuild or additional Python packages are needed. The new script adds three
collision objects through `/apply_planning_scene`, calls `/plan_kinematic_path`,
and publishes the resulting trajectory to `/display_planned_path`. It never
calls a trajectory execution action. The arm's current state remains unchanged.

In RViz's MotionPlanning display, enable **Scene Geometry** and
**Planned Path → Show Robot Visual** to see a preview. The saved demo settings
now hide previews and goal/start overlays by default, disable Loop Animation,
and show the current Scene Robot at full opacity.
The red cap, yellow housing, and grey panel should appear, together with the
planned approach animation. To see labels and a green approach-direction arrow,
use **Add → By topic → /button_demo/markers → MarkerArray**. Set the display's
Durability Policy to **Transient Local** if the existing markers do not appear.
Keep terminal 2 open so late subscribers can receive the preview and markers.
The green arrow indicates the button axis; it is not an executed press path.

The script's preview is separate from the RViz panel's internally stored plan.
To execute a different goal on the mock arm using the RViz panel, select `arm`,
set Start State to `<current>`, position the goal marker, and use that panel's
**Plan**, then **Execute**. Do not assume its Execute button executes the
trajectory published by this script.

## Geometry and coordinates

All distances are metres in `base_link`. The button faces the robot along -X;
pressing would move the tool in +X.

| Item | Default |
| --- | --- |
| Button front-face center | X=0.50, Y=0.00, Z=0.30 |
| Button cap | radius 0.025, thickness 0.016 |
| Yellow housing | 0.03 × 0.09 × 0.09 |
| Mounting panel | 0.04 × 0.22 × 0.22 |
| Wrist approach pose | X=0.30, Y=0.00, Z=0.30 |
| Approach orientation | local tool +Z aligned with base +X |

The launcher's current `tcp_link` is at the wrist flange, not the fingertips.
The 0.20 m stand-off is measured from the button face to that flange and includes
space for the gripper. It is an illustrative approach offset, not a measured
pressing distance. The goal tolerance is 3 mm and about 0.03 rad per orientation
axis. Collision checks still apply to the gripper and the button.

To reposition the button, stop terminal 2 with Ctrl+C and run, for example:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_button_scene.sh \
  --x 0.50 --y 0.05 --z 0.30 --stand-off 0.20
```

The same object IDs are updated instead of duplicating the button. Arbitrary
positions may be unreachable or cause collisions; a failed plan is reported
without executing anything. `--scene-only` adds geometry without requesting a
plan. `--once` exits after adding/planning, for headless verification; keep the
normal script running for reliable late RViz subscriptions.

The objects remain in MoveIt when terminal 2 closes. To remove this demo's
three collision objects, first stop terminal 2 and run:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_button_scene.sh --remove
```

Restarting MoveIt also clears this unsaved scene. Restart/remove the MarkerArray
display to clear retained labels if needed.

## Make the simulated gripper touch the button

Keep `run_moveit_demo.sh` running in terminal 1. Stop the older button-scene
preview in terminal 2 with Ctrl+C, to avoid competing trajectory displays.
Close the simulated gripper if it is open: select the `gripper` group and
`gripper_close`, then Plan and Execute in RViz.

Preview the approach and touch:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_touch_button.sh
```

After viewing the preview, stop that script with Ctrl+C and run this to move
the mock arm:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_touch_button.sh --execute
```

The script uses the same `--x`, `--y`, `--z` button-face coordinates as the
scene script. It creates/updates the button scene itself, so the old scene
script does not need to be running. No rebuild is needed.

The closed-finger collision meshes extend 0.1425 m along local +Z from `link6`:
0.0045 m flange-to-gripper-base plus 0.138 m to the front finger plane. With
tool +Z pointing along base +X, the default wrist contact pose is
`(0.3575, 0, 0.30)` m. The pre-contact pose is 20 mm back from this.
This offset is specific to the supplied standard Piper gripper model; it is
not a calibration of a real fingertip or another tool.

The script plans the free-space approach, then a straight Cartesian contact
segment at a requested maximum wrist speed of 0.01 m/s. It requires a complete
Cartesian path and checks the fingertip endpoint with forward kinematics.
Only `gripper_link1` / `gripper_link2` contact with `button_demo_cap` is temporarily
allowed. Panel, housing, and other robot collision checks remain enabled; the
original collision matrix is restored after planning/execution. Do not run
other scene-editing or motion scripts simultaneously with this demo.

Execution is opt-in and requires the controller manager to report
`mock_components/GenericSystem`. This is a demonstration for the existing
isolated mock setup, not a physical-arm control script. It leaves the mock
fingers at the button surface; it does not press through it, latch a switch,
or automatically retract. The exact-contact state is subject to mesh/numerical
tolerances; if a later plan reports the start state in collision, restart the
mock demo to return to its initial state before trying again.

Validation on the Jetson: both planned stages executed successfully on mock
hardware. The final modeled fingertip center was within 0.002 mm of
`(0.50, 0, 0.30)` m. The endpoint was collision-free under the restored rules,
and the temporary button collision entry was removed. This numerical agreement
is not real-world accuracy or proof of pressing force.

## Push, hold for 3 seconds, and return

Keep `run_moveit_demo.sh` running. Close older button/touch preview scripts with
Ctrl+C in their own terminals. Do not send other motion or scene-edit commands
while the sequence runs. The simulated gripper must be closed.

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_press_button.sh --execute --hold 3
```

No rebuild is required. The command runs once and exits after completion:

1. Save the current six arm joint positions as the initial pose.
2. Plan all five moving stages before starting any execution.
3. Approach with 20 mm fingertip clearance, then move straight to contact.
4. Push a further 5 mm into the virtual button travel.
5. Confirm the pressed pose from feedback and hold position for 3 seconds.
6. Retract to the clearance pose and return to the saved initial joints.
7. Restore the virtual button cap and original collision rules.

The initial pose is the pose when this command starts, not necessarily the
named `home` pose. If you want to finish at home, move to `home` in RViz before
running the sequence. The gripper stays closed throughout.

The cap changes to its depressed position after the push finishes and resets
after retraction. This is a scripted geometric demonstration: the cap is not
physics-driven, force is not measured, and E-stop latch/reset behavior is not
modeled. A 3-second positional hold in mock hardware is not a physical force hold.

To preview instead of executing, omit `--execute`:

```bash
bash /home/mppi_zd/AgileX_piper/scripts/run_press_button.sh --hold 3
```

For execution, watch the **Scene Robot** in RViz. Under MotionPlanning, turn off
**Planning Request → Query Goal State** and **Query Start State**, turn off
**Planned Path → Show Robot Visual** and **Loop Animation**, and turn on
**Scene Robot → Show Robot Visual** (Robot Alpha 1.0). The orange goal marker
and looping grey trajectory preview are not live robot feedback. Do not click
RViz Execute for this sequence; the script executes its own trajectories.
For preview-only use, turn **Planned Path → Show Robot Visual** back on.
The preview animation speed may differ from execution timing because of RViz's
State Display Time setting; the executed hold uses a monotonic 3-second timer.

Options: `--x`, `--y`, `--z` move the virtual button; `--press-depth 0.005`
changes virtual travel (limited to 8 mm for this housing); `--hold 3` changes
the dwell (0–60 seconds). Arbitrary target positions may be unreachable.
During execution, `/button_demo/stage` reports the stage and
`/button_demo/pressed` reports the scripted pressed state.

On failure, the active execution action is canceled when possible; the script
does not start an unplanned automatic recovery motion. Inspect the mock arm
before retrying. The return guarantee applies to successful sequence completion.

Validation: the full mock sequence completed with a 3.000-second measured hold
and maximum return joint error of 0.000092 rad. The cap and collision rules were
restored. No physical arm was controlled.

## Moving from approach to pressing

1. Model a pushing tip and calibrate its TCP. Replace the flange-based stand-off
   with a tip-based clearance.
2. Plan a collision-free approach to a pose just in front of the button.
3. Create a short straight Cartesian segment along the button axis. Require a
   complete valid path before executing it. Keep collisions enabled for the
   panel and robot; allow only the intended tip/button contact when appropriate.
4. Add contact feedback and force/travel/time limits. MoveIt planning alone does
   not simulate button compliance or establish that the switch activated.
5. Verify button activation, then retract if the stop state permits it.

For a physics version, keep MoveIt and RViz, and replace mock hardware with
Gazebo-connected controllers. Model the cap as a limited prismatic joint along
its pressing axis, with measured spring/damping properties and contact geometry.
Use simulated joint displacement/contact to drive a button-state signal; a
latching E-stop additionally needs latch/reset logic. Match the Gazebo geometry
and poses to MoveIt's planning scene. Merely spawning a model in Gazebo does
not automatically add an obstacle to MoveIt.

This virtual button is a task target. It is not an implementation of the real
Piper's safety stop or a safety-rated machine emergency-stop system.

## Validation

On this Jetson, the default scene update and OMPL approach plan succeeded against
the running mock Piper, producing a 39-point trajectory. No execution was
requested. RViz appearance and physical button contact have not been validated.

- [MoveIt planning-scene API](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)
- [Gazebo sensors and contact tutorial](https://gazebosim.org/docs/harmonic/sensors/)
