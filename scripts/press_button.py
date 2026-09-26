#!/usr/bin/env python3
"""Mock Piper: approach, press, hold, retract, return to captured starting joints."""
import argparse
import copy
import math
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene,
    GetPositionFK,
)
from std_msgs.msg import Bool, String
from button_scene import add_scene, call, plan, pose
from touch_button import TIP_Z, CAP, apply_matrix, contact_matrix, execute, final_state


ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]


def get_scene(node):
    request = GetPlanningScene.Request()
    request.components.components = (PlanningSceneComponents.ROBOT_STATE |
                                     PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    return call(node, GetPlanningScene, '/get_planning_scene', request).scene


def cartesian(node, start, target):
    request = GetCartesianPath.Request()
    request.header.frame_id = 'base_link'
    request.start_state = start
    request.group_name = 'arm'
    request.link_name = 'link6'
    request.waypoints = [target]
    request.max_step = 0.001
    request.jump_threshold = 5.0
    request.revolute_jump_threshold = 0.15
    request.avoid_collisions = True
    request.max_velocity_scaling_factor = 0.05
    request.max_acceleration_scaling_factor = 0.05
    request.cartesian_speed_limited_link = 'link6'
    request.max_cartesian_speed = 0.01
    result = call(node, GetCartesianPath, '/compute_cartesian_path', request)
    if result.error_code.val != 1 or result.fraction < 0.999999:
        raise RuntimeError(f'Incomplete Cartesian path: {result.fraction:.1%}, code {result.error_code.val}; sequence aborted')
    if not result.solution.joint_trajectory.points:
        raise RuntimeError('Cartesian planner returned no trajectory')
    return result.solution


def return_plan(node, start, initial):
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = 'arm'
    motion.pipeline_id = 'ompl'
    motion.start_state = start
    motion.allowed_planning_time = 5.0
    motion.num_planning_attempts = 5
    motion.max_velocity_scaling_factor = 0.1
    motion.max_acceleration_scaling_factor = 0.1
    initial_joints = dict(zip(initial.joint_state.name, initial.joint_state.position))
    goal = Constraints()
    for name in ARM_JOINTS:
        goal.joint_constraints.append(JointConstraint(
            joint_name=name, position=initial_joints[name],
            tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0))
    motion.goal_constraints = [goal]
    result = call(node, GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response
    if result.error_code.val != 1:
        raise RuntimeError(f'Return planning failed: {result.error_code.val}; nothing executed')
    return result.trajectory


def tip_position(node, state, tip_offset=TIP_Z):
    request = GetPositionFK.Request()
    request.header.frame_id = 'base_link'
    request.fk_link_names = ['link6']
    request.robot_state = state
    result = call(node, GetPositionFK, '/compute_fk', request)
    if result.error_code.val != 1:
        raise RuntimeError('Fingertip FK failed')
    p = result.pose_stamped[0].pose
    q = p.orientation
    return (p.position.x + 2 * (q.x*q.z + q.w*q.y) * tip_offset,
            p.position.y + 2 * (q.y*q.z - q.w*q.x) * tip_offset,
            p.position.z + (1 - 2 * (q.x*q.x + q.y*q.y)) * tip_offset)


def check_joints(actual, expected, tolerance=0.001):
    a = dict(zip(actual.joint_state.name, actual.joint_state.position))
    e = dict(zip(expected.joint_state.name, expected.joint_state.position))
    error = max(abs(a[name] - e[name]) for name in ARM_JOINTS)
    if error > tolerance:
        raise RuntimeError(f'Robot differs from expected pose by {error:.6f} rad; sequence aborted')
    return error


def move_cap(node, args, depth):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    obj = CollisionObject()
    obj.header.frame_id = 'base_link'
    obj.id = CAP
    obj.operation = CollisionObject.MOVE
    obj.primitive_poses = [pose(args.x + 0.008 + depth, args.y, args.z, True)]
    request.scene.world.collision_objects = [obj]
    if not call(node, ApplyPlanningScene, '/apply_planning_scene', request).success:
        raise RuntimeError('Could not update the virtual button cap')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name, default in [('x', 0.5), ('y', 0.0), ('z', 0.3)]:
        parser.add_argument(f'--{name}', type=float, default=default)
    parser.add_argument('--hold', type=float, default=3.0, help='Hold time in seconds after reaching the pressed pose')
    parser.add_argument('--press-depth', type=float, default=0.005, help='Virtual button travel in metres (up to 0.008)')
    parser.add_argument('--execute', action='store_true', help='Run the sequence on verified mock hardware')
    parser.add_argument('--once', action='store_true', help='Exit after a preview-only check')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z, args.hold, args.press_depth)):
        parser.error('All numeric arguments must be finite')
    if not 0 < args.press_depth <= 0.008:
        parser.error('--press-depth must be > 0 and <= 0.008 m for this housing')
    if not 0 <= args.hold <= 60:
        parser.error('--hold must be between 0 and 60 seconds')
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        parser.error('Use run_press_button.sh in the isolated mock demo')
    args.remove = False
    rclpy.init()
    node = rclpy.create_node('piper_press_button')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    stage_pub = node.create_publisher(String, '/button_demo/stage', qos)
    pressed_pub = node.create_publisher(Bool, '/button_demo/pressed', qos)
    def stage(text):
        print(text, flush=True)
        stage_pub.publish(String(data=text))
    try:
        hardware = call(node, ListHardwareComponents,
                        '/controller_manager/list_hardware_components', ListHardwareComponents.Request())
        if not hardware.component or any(c.plugin_name != 'mock_components/GenericSystem' for c in hardware.component):
            raise RuntimeError('This sequence supports mock_components/GenericSystem only')
        scene = get_scene(node)
        initial = copy.deepcopy(scene.robot_state)
        initial.is_diff = False
        joints = dict(zip(initial.joint_state.name, initial.joint_state.position))
        if not all(name in joints for name in ARM_JOINTS):
            raise RuntimeError('Expected six Piper arm joints')
        if abs(joints.get('gripper', 1.0)) > 0.001:
            raise RuntimeError('Close the simulated gripper first using gripper_close in RViz')
        print('Captured starting arm joints:', [round(joints[n], 6) for n in ARM_JOINTS], flush=True)
        add_scene(node, args)
        original_matrix = copy.deepcopy(scene.allowed_collision_matrix)
        cap_changed = False
        succeeded = False
        try:
            # Allow only the intended contact pair, also allowing a start at touch.
            apply_matrix(node, contact_matrix(original_matrix))
            contact = pose(args.x - TIP_Z, args.y, args.z, True)
            approach = copy.deepcopy(contact)
            approach.position.x -= 0.02
            pressed = copy.deepcopy(contact)
            pressed.position.x += args.press_depth
            stage('Planning the complete sequence before execution...')
            display = plan(node, approach, link_name='link6', position_tolerance=0.0001,
                           orientation_tolerance=0.001, start_state=initial)
            after_approach = final_state(initial, display.trajectory[0])
            touch = cartesian(node, after_approach, contact)
            after_touch = final_state(after_approach, touch)
            push = cartesian(node, after_touch, pressed)
            after_push = final_state(after_touch, push)
            retract = cartesian(node, after_push, approach)
            after_retract = final_state(after_push, retract)
            # Keep only the fingertip/cap exception, including when the saved
            # initial pose is already at the button; all other checks stay on.
            returning = return_plan(node, after_retract, initial)
            if math.dist(tip_position(node, after_push), (args.x + args.press_depth, args.y, args.z)) > 0.0005:
                raise RuntimeError('Pressed endpoint differs from the button target')
            check_joints(final_state(after_retract, returning), initial, tolerance=0.0005)
            stage(f'All paths complete: approach → touch → push {args.press_depth*1000:.1f} mm → hold {args.hold:.1f} s → retract → initial pose.')
            if not args.execute:
                # Add a stationary trajectory to represent the dwell in the preview.
                from moveit_msgs.msg import RobotTrajectory
                from trajectory_msgs.msg import JointTrajectoryPoint
                from builtin_interfaces.msg import Duration
                hold = RobotTrajectory()
                hold.joint_trajectory.joint_names = push.joint_trajectory.joint_names
                positions = list(push.joint_trajectory.points[-1].positions)
                ns = round(args.hold * 1e9)
                if ns > 0:
                    hold.joint_trajectory.points = [
                        JointTrajectoryPoint(positions=positions, time_from_start=Duration()),
                        JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=ns//1000000000, nanosec=ns%1000000000)),
                    ]
                display.trajectory += [touch, push] + ([hold] if ns > 0 else []) + [retract, returning]
                preview_pub = node.create_publisher(DisplayTrajectory, '/display_planned_path', qos)
                preview_pub.publish(display)
                stage('Preview only. Run again with --execute to move the mock robot.')
            else:
                check_joints(get_scene(node).robot_state, initial)
                pressed_pub.publish(Bool(data=False))
                stage('1/6 Approach')
                execute(node, display.trajectory[0])
                stage('2/6 Touch')
                execute(node, touch)
                stage('3/6 Push')
                execute(node, push)
                actual = get_scene(node).robot_state
                if math.dist(tip_position(node, actual), (args.x + args.press_depth, args.y, args.z)) > 0.001:
                    raise RuntimeError('Feedback did not reach the pressed pose')
                cap_changed = True
                move_cap(node, args, args.press_depth)
                pressed_pub.publish(Bool(data=True))
                stage(f'4/6 Hold for {args.hold:.1f} seconds')
                began = time.monotonic()
                while time.monotonic() - began < args.hold:
                    rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, args.hold - (time.monotonic() - began))))
                elapsed = time.monotonic() - began
                check_joints(get_scene(node).robot_state, after_push)
                print(f'Hold completed: {elapsed:.3f} seconds.', flush=True)
                stage('5/6 Retract')
                execute(node, retract)
                move_cap(node, args, 0.0)
                cap_changed = False
                pressed_pub.publish(Bool(data=False))
                stage('6/6 Return to captured initial pose')
                execute(node, returning)
                error = check_joints(get_scene(node).robot_state, initial)
                stage(f'Done. Initial pose restored; maximum joint error {error:.6f} rad.')
                succeeded = True
        finally:
            # Reset visual state and restore only after stopping active execution.
            # execute() cancels its action on errors or interruption.
            if rclpy.ok():
                if cap_changed:
                    move_cap(node, args, 0.0)
                apply_matrix(node, original_matrix)
                pressed_pub.publish(Bool(data=False))
                if args.execute and not succeeded:
                    stage('Aborted. No automatic recovery motion; inspect the simulated robot before retrying.')
        if not args.execute and not args.once:
            print('Ctrl+C closes the preview publisher.', flush=True)
            rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
