#!/usr/bin/env python3
"""Add a virtual E-stop and preview a collision-checked approach. Never execute."""
import argparse
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Point, Pose, Vector3
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, ObjectColor,
    OrientationConstraint, PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


OBJECT_IDS = ('button_demo_panel', 'button_demo_housing', 'button_demo_cap')


def pose(x, y, z, along_x=False):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w = math.sqrt(0.5) if along_x else 1.0
    p.orientation.y = math.sqrt(0.5) if along_x else 0.0
    return p


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'{name} unavailable. Start the matching mock or real Piper launch first.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done():
            raise RuntimeError(f'{name} timed out')
        return future.result()
    finally:
        node.destroy_client(client)


def add_scene(node, args):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    specifications = [
        (OBJECT_IDS[0], SolidPrimitive.BOX, [0.04, 0.22, 0.22],
         pose(args.x + 0.06, args.y, args.z), (0.35, 0.38, 0.42)),
        (OBJECT_IDS[1], SolidPrimitive.BOX, [0.03, 0.09, 0.09],
         pose(args.x + 0.025, args.y, args.z), (1.0, 0.8, 0.0)),
        # Cylinders use [height, radius], with their local Z axis rotated to X.
        (OBJECT_IDS[2], SolidPrimitive.CYLINDER, [0.016, 0.025],
         pose(args.x + 0.008, args.y, args.z, True), (0.9, 0.02, 0.02)),
    ]
    for name, kind, dimensions, location, rgb in specifications:
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = name
        obj.operation = CollisionObject.REMOVE if args.remove else CollisionObject.ADD
        if not args.remove:
            obj.primitives = [SolidPrimitive(type=kind, dimensions=dimensions)]
            obj.primitive_poses = [location]
            color = ObjectColor(id=name)
            color.color.r, color.color.g, color.color.b = rgb
            color.color.a = 1.0
            request.scene.object_colors.append(color)
        request.scene.world.collision_objects.append(obj)
    response = call(node, ApplyPlanningScene, '/apply_planning_scene', request)
    if not response.success:
        raise RuntimeError('MoveIt rejected the planning-scene update')


def make_markers(args, target):
    markers = MarkerArray()
    for i, text, location in (
        (0, 'E-STOP (virtual)', pose(args.x, args.y, args.z + 0.15)),
        (1, 'Approach: wrist TCP', pose(target.position.x, args.y, args.z + 0.08)),
    ):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.ns = 'button_demo'
        marker.id = i
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = location
        marker.scale.z = 0.025
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = text
        markers.markers.append(marker)
    arrow = Marker()
    arrow.header.frame_id = 'base_link'
    arrow.ns = 'button_demo'
    arrow.id = 2
    arrow.type = Marker.ARROW
    arrow.pose.orientation.w = 1.0
    arrow.points = [target.position, Point(x=args.x, y=args.y, z=args.z)]
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.005, 0.012, 0.02
    arrow.color.g = arrow.color.a = 1.0
    markers.markers.append(arrow)
    return markers


def plan(node, target, *, link_name="tcp_link", position_tolerance=0.003, orientation_tolerance=0.03, start_state=None):
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = 'arm'
    motion.pipeline_id = 'ompl'
    motion.allowed_planning_time = 5.0
    motion.num_planning_attempts = 5
    motion.max_velocity_scaling_factor = 0.1
    motion.max_acceleration_scaling_factor = 0.1
    if start_state is None:
        motion.start_state.is_diff = True
    else:
        motion.start_state = start_state
    motion.workspace_parameters.header.frame_id = 'base_link'
    motion.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-0.1)
    motion.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.2)
    goal = Constraints()
    position = PositionConstraint()
    position.header.frame_id = 'base_link'
    position.link_name = link_name
    position.weight = 1.0
    position.constraint_region.primitives = [
        SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
    ]
    position.constraint_region.primitive_poses = [target]
    goal.position_constraints = [position]
    orientation = OrientationConstraint()
    orientation.header.frame_id = 'base_link'
    orientation.link_name = link_name
    orientation.orientation = target.orientation
    orientation.absolute_x_axis_tolerance = orientation_tolerance
    orientation.absolute_y_axis_tolerance = orientation_tolerance
    orientation.absolute_z_axis_tolerance = orientation_tolerance
    orientation.weight = 1.0
    goal.orientation_constraints = [orientation]
    motion.goal_constraints = [goal]
    result = call(node, GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response
    if result.error_code.val != 1:
        raise RuntimeError(
            f'Approach planning failed (MoveIt code {result.error_code.val}). '
            'Check the current robot state, target reachability, and collisions.'
        )
    display = DisplayTrajectory()
    display.model_id = 'agx_arm'
    display.trajectory_start = result.trajectory_start
    display.trajectory = [result.trajectory]
    return display


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.50, help='Button face X, metres in base_link')
    parser.add_argument('--y', type=float, default=0.0, help='Button face Y')
    parser.add_argument('--z', type=float, default=0.30, help='Button face Z')
    parser.add_argument('--stand-off', type=float, default=0.20,
                        help='Distance from button face to wrist TCP, including gripper length')
    parser.add_argument('--scene-only', action='store_true', help='Add objects without planning')
    parser.add_argument('--remove', action='store_true', help='Remove only this demo\'s objects')
    parser.add_argument('--once', action='store_true', help='Exit after setup/plan (for headless checks)')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z, args.stand_off)):
        parser.error('Coordinates and stand-off must be finite')
    if args.stand_off <= 0:
        parser.error('--stand-off must be positive')
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        parser.error('Use run_button_scene.sh; this demo is scoped to ROS domain 77')
    rclpy.init()
    node = rclpy.create_node('piper_button_scene')
    try:
        add_scene(node, args)
        if args.remove:
            print('Removed the three virtual button collision objects.', flush=True)
            return
        target = pose(args.x - args.stand_off, args.y, args.z, True)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        marker_pub = node.create_publisher(MarkerArray, '/button_demo/markers', qos)
        marker_pub.publish(make_markers(args, target))
        print(f'Button face in base_link: ({args.x:.3f}, {args.y:.3f}, {args.z:.3f}) m', flush=True)
        print(f'Wrist approach: ({target.position.x:.3f}, {args.y:.3f}, {args.z:.3f}) m; tool +Z points along base +X.', flush=True)
        if not args.scene_only:
            display = plan(node, target)
            publisher = node.create_publisher(DisplayTrajectory, '/display_planned_path', qos)
            publisher.publish(display)
            print(f'Preview ready: {len(display.trajectory[0].joint_trajectory.points)} trajectory points. No execution requested.', flush=True)
        if not args.once:
            print('Keep this terminal open for RViz. Ctrl+C exits; scene objects remain until --remove or MoveIt restarts.', flush=True)
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
