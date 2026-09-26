#!/usr/bin/env python3
"""Plan fingertip touch, with optional execution on the mock Piper only."""
import argparse
import copy
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AllowedCollisionEntry, DisplayTrajectory, PlanningSceneComponents
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionFK,
)
from button_scene import add_scene, call, plan, pose

# From the supplied Piper URDF and finger collision meshes, for closed fingers:
# flange -> gripper_base: +0.0045 Z; finger origin: +0.138 Z.
# Finger mesh local maximum Y is 0; its joint rotates local Y into flange Z.
TIP_Z = 0.1425
CAP = 'button_demo_cap'
FINGERS = ('gripper_link1', 'gripper_link2')


def final_state(start, trajectory):
    state = copy.deepcopy(start)
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    values.update(zip(trajectory.joint_trajectory.joint_names,
                      trajectory.joint_trajectory.points[-1].positions))
    state.joint_state.name = list(values)
    state.joint_state.position = list(values.values())
    state.joint_state.velocity = []
    state.joint_state.effort = []
    state.is_diff = False
    return state


def apply_matrix(node, matrix):
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    req.scene.robot_state.is_diff = True
    req.scene.allowed_collision_matrix = matrix
    if not call(node, ApplyPlanningScene, '/apply_planning_scene', req).success:
        raise RuntimeError('Could not update/restore contact collision rules')


def contact_matrix(original, cap_name=CAP):
    matrix = copy.deepcopy(original)
    for name in (cap_name, *FINGERS):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
    cap = matrix.entry_names.index(cap_name)
    for name in FINGERS:
        finger = matrix.entry_names.index(name)
        matrix.entry_values[cap].enabled[finger] = True
        matrix.entry_values[finger].enabled[cap] = True
    return matrix


def execute(node, trajectory, timeout=45):
    client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    handle = None
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Trajectory executor unavailable')
        future = client.send_goal_async(ExecuteTrajectory.Goal(
            trajectory=trajectory, controller_names=['arm_controller']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError('Execution goal acknowledgement timed out')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Execution rejected')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Execution timed out')
        if future.result().result.error_code.val != 1:
            raise RuntimeError(f'Execution failed: {future.result().result.error_code.val}')
        handle = None
    finally:
        if handle is not None and rclpy.ok():
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        client.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.50)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.30)
    parser.add_argument('--execute', action='store_true', help='Execute on verified mock hardware')
    parser.add_argument('--once', action='store_true', help='Exit after planning/execution')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z)):
        parser.error('Coordinates must be finite')
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        parser.error('Use run_touch_button.sh in the dedicated mock demo')
    args.remove = False
    rclpy.init()
    node = rclpy.create_node('piper_touch_button')
    try:
        hardware = call(node, ListHardwareComponents,
                        '/controller_manager/list_hardware_components', ListHardwareComponents.Request())
        if not hardware.component or any(c.plugin_name != 'mock_components/GenericSystem' for c in hardware.component):
            raise RuntimeError('This touch demo requires mock_components/GenericSystem hardware')
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ROBOT_STATE |
                                     PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        scene = call(node, GetPlanningScene, '/get_planning_scene', req).scene
        joints = dict(zip(scene.robot_state.joint_state.name, scene.robot_state.joint_state.position))
        if 'gripper' not in joints or abs(joints['gripper']) > 0.001:
            raise RuntimeError('Close the simulated gripper first: RViz group gripper, goal gripper_close, Plan and Execute.')
        add_scene(node, args)
        contact = pose(args.x - TIP_Z, args.y, args.z, True)
        approach = copy.deepcopy(contact)
        approach.position.x -= 0.02
        display = plan(node, approach, link_name='link6',
                       position_tolerance=0.0001, orientation_tolerance=0.001)
        start = final_state(display.trajectory_start, display.trajectory[0])
        original_matrix = scene.allowed_collision_matrix
        try:
            apply_matrix(node, contact_matrix(original_matrix))
            req = GetCartesianPath.Request()
            req.header.frame_id = 'base_link'
            req.start_state = start
            req.group_name = 'arm'
            req.link_name = 'link6'
            req.waypoints = [contact]
            req.max_step = 0.001
            req.jump_threshold = 5.0
            req.revolute_jump_threshold = 0.15
            req.avoid_collisions = True
            req.max_velocity_scaling_factor = 0.05
            req.max_acceleration_scaling_factor = 0.05
            req.cartesian_speed_limited_link = 'link6'
            req.max_cartesian_speed = 0.01
            result = call(node, GetCartesianPath, '/compute_cartesian_path', req)
            if result.error_code.val != 1 or result.fraction < 0.999999:
                raise RuntimeError(f'Incomplete contact path: {result.fraction:.1%}, code {result.error_code.val}; nothing executed')
            # Forward-kinematics check of the modeled fingertip face at the endpoint.
            fk = GetPositionFK.Request()
            fk.header.frame_id = 'base_link'
            fk.fk_link_names = ['link6']
            fk.robot_state = final_state(start, result.solution)
            endpoint = call(node, GetPositionFK, '/compute_fk', fk)
            if endpoint.error_code.val != 1:
                raise RuntimeError('Contact endpoint FK failed')
            p = endpoint.pose_stamped[0].pose
            q = p.orientation
            tip = (p.position.x + 2 * (q.x*q.z + q.w*q.y) * TIP_Z,
                   p.position.y + 2 * (q.y*q.z - q.w*q.x) * TIP_Z,
                   p.position.z + (1 - 2 * (q.x*q.x + q.y*q.y)) * TIP_Z)
            error = math.dist(tip, (args.x, args.y, args.z))
            if error > 0.0005:
                raise RuntimeError(f'Modeled contact error too large: {error*1000:.3f} mm')
            display.trajectory.append(result.solution)
            publisher = node.create_publisher(DisplayTrajectory, '/display_planned_path',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            publisher.publish(display)
            print(f'Approach then 20 mm straight touch: Cartesian path {result.fraction:.0%}; fingertip model error {error*1000:.3f} mm.', flush=True)
            if args.execute:
                execute(node, display.trajectory[0])
                execute(node, result.solution)
                print('Mock arm reached the button face. This is geometric touch, not force/contact simulation.', flush=True)
            else:
                print('Preview only. Use --execute to move the mock arm.', flush=True)
        finally:
            apply_matrix(node, original_matrix)
        if not args.once:
            print('Ctrl+C closes this preview publisher.', flush=True)
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
