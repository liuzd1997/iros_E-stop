#!/usr/bin/env python3
"""Detect, identify, press/hold/verify every target, then return once. Mock only."""
import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, ObjectColor, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from button_perception import Tracker, detect_buttons, orientation, synthetic_rgbd
from button_scene import call, plan
from press_button import ARM_JOINTS, cartesian, check_joints, return_plan, tip_position
from touch_button import TIP_Z, apply_matrix, contact_matrix, execute, final_state

ROOT = Path(__file__).resolve().parents[1]


def scene(node):
    request = GetPlanningScene.Request()
    request.components.components = (PlanningSceneComponents.ROBOT_STATE |
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
    return call(node, GetPlanningScene, '/get_planning_scene', request).scene


def target_pose(target, axial_offset):
    p = np.asarray(target['position']) + axial_offset * np.asarray(target['direction'])
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, p)
    q = orientation(target['direction'])
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, q)
    return result


def object_id(key, part):
    return f'multi_{key}_{part}'


def update_scene(node, inventory, previous):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    for old in previous.world.collision_objects:
        if old.id.startswith('multi_button_') or old.id in ('button_demo_cap', 'button_demo_housing', 'button_demo_panel'):
            request.scene.world.collision_objects.append(CollisionObject(id=old.id, operation=CollisionObject.REMOVE))
    for key, target in inventory.items():
        for part, kind, dimensions, offset, rgb in (
            ('panel', SolidPrimitive.BOX, [.22, .22, .04], .06, (.35, .38, .42)),
            ('housing', SolidPrimitive.BOX, [.09, .09, .03], .025, (1., .8, 0.)),
            ('cap', SolidPrimitive.CYLINDER, [.016, .025], .008, (.9, .02, .02)),
        ):
            obj = CollisionObject(id=object_id(key, part), operation=CollisionObject.ADD)
            obj.header.frame_id = 'base_link'
            obj.primitives = [SolidPrimitive(type=kind, dimensions=dimensions)]
            obj.primitive_poses = [target_pose(target, offset)]
            request.scene.world.collision_objects.append(obj)
            color = ObjectColor(id=obj.id)
            color.color.r, color.color.g, color.color.b = rgb
            color.color.a = 1.
            request.scene.object_colors.append(color)
    if not call(node, ApplyPlanningScene, '/apply_planning_scene', request).success:
        raise RuntimeError('Failed to add the complete button inventory to MoveIt')


def move_cap(node, key, target, depth):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    obj = CollisionObject(id=object_id(key, 'cap'), operation=CollisionObject.MOVE)
    obj.header.frame_id = 'base_link'
    obj.primitive_poses = [target_pose(target, .008+depth)]
    request.scene.world.collision_objects = [obj]
    if not call(node, ApplyPlanningScene, '/apply_planning_scene', request).success:
        raise RuntimeError(f'Failed to move virtual cap {key}')


def verify_pressed(node, key, target, depth):
    observed = scene(node)
    error = np.linalg.norm(np.asarray(tip_position(node, observed.robot_state)) -
                           (target['position'] + depth*target['direction']))
    cap = next((obj for obj in observed.world.collision_objects if obj.id == object_id(key, 'cap')), None)
    if cap is None or not cap.primitive_poses:
        raise RuntimeError(f'{key}: cap missing from scene feedback')
    p = cap.primitive_poses[0].position
    cap_error = np.linalg.norm(np.array([p.x, p.y, p.z]) -
                              (target['position'] + (.008+depth)*target['direction']))
    if error > .001 or cap_error > .0001:
        raise RuntimeError(f'{key}: mock verification failed (tip {error:.6f} m, cap {cap_error:.6f} m)')
    return dict(method='mock_joint_FK_and_scene_geometry', tip_error_m=float(error), cap_error_m=float(cap_error))


class RunReport:
    def __init__(self, node, args):
        self.node = node
        self.path = Path(args.report).resolve() if args.report else ROOT/'outputs'/f'multi_run_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")}.json'
        self.data = dict(source=args.source, mode='execute_mock' if args.execute else 'plan_only',
                         started_at=datetime.now(timezone.utc).isoformat(), result='running',
                         initial_pose_restored=False, buttons={}, events=[])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, '/button_demo/status', qos)
        self.markers = node.create_publisher(MarkerArray, '/button_demo/targets', qos)

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.data, indent=2, allow_nan=False)+'\n')
        tmp.replace(self.path)
        if rclpy.ok():
            self.publisher.publish(String(data=json.dumps(self.data, allow_nan=False)))

    def event(self, stage, key=None):
        self.data['events'].append(dict(stage=stage, button=key, monotonic=time.monotonic()))
        print(f'{key + ": " if key else ""}{stage}', flush=True)
        self.save()
        self.show_labels()

    def show_labels(self):
        array = MarkerArray()
        for index, (key, value) in enumerate(self.data['buttons'].items()):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.ns = 'multi_button_status'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = value['position']
            marker.pose.position.z += .09
            marker.pose.orientation.w = 1.
            marker.scale.z = .022
            marker.color.a = 1.
            status = value['status']
            marker.color.r = 0. if status == 'completed' else 1.
            marker.color.g = 0. if status == 'failed' else 1.
            marker.text = f'{key}: {status}'
            array.markers.append(marker)
        if rclpy.ok():
            self.markers.publish(array)


def acquire_synthetic(node):
    inputs = synthetic_rgbd()
    cv2.imwrite(str(ROOT/'outputs'/'synthetic_buttons.png'), inputs[0])
    tracker = Tracker()
    detections = detect_buttons(*inputs)
    for _ in range(5):
        tracker.update(detections, time.monotonic())
    targets = tracker.ready(time.monotonic())
    annotated = inputs[0].copy()
    for detection in detections:
        key = min(targets, key=lambda key: np.linalg.norm(targets[key]['position']-detection.position))
        u, v = map(lambda x: int(round(x)), detection.pixel)
        cv2.circle(annotated, (u, v), 29, (0, 255, 0), 2)
        cv2.putText(annotated, key, (u-45, v-55), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
    cv2.putText(annotated, 'SYNTHETIC RGB-D INPUT', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    cv2.imwrite(str(ROOT/'outputs'/'synthetic_detections.png'), annotated)
    message = Image()
    message.header.frame_id = 'synthetic_color_optical_frame'
    message.header.stamp = node.get_clock().now().to_msg()
    message.height, message.width = annotated.shape[:2]
    message.encoding = 'bgr8'
    message.step = message.width*3
    message.data = annotated.tobytes()
    publisher = node.create_publisher(Image, '/button_demo/synthetic_image', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(message)
    return targets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=['synthetic', 'camera'], default='synthetic')
    parser.add_argument('--expected-count', type=int, help='Required inventory size; mandatory for camera input')
    parser.add_argument('--config', default=str(ROOT/'config/d435i_buttons.yaml'))
    parser.add_argument('--calibration', help='Measured fixed-camera transform YAML; otherwise require valid tf2')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--hold', type=float, default=3.)
    parser.add_argument('--press-depth', type=float, default=.005)
    parser.add_argument('--report', help='Optional result JSON path')
    parser.add_argument('--once', action='store_true', help='Exit after planning; otherwise keep preview available')
    parser.add_argument('--keep-open', action='store_true', help='Keep final status and synthetic image available after execution; never repeats motion')
    args = parser.parse_args()
    if not math.isfinite(args.hold) or not 0 <= args.hold <= 60:
        parser.error('--hold must be 0–60 seconds')
    if not math.isfinite(args.press_depth) or not 0 < args.press_depth <= .008:
        parser.error('--press-depth must be >0 and <=0.008 m for this button model')
    if args.source == 'camera' and args.expected_count is None:
        parser.error('--expected-count is required so missing buttons cannot silently be ignored')
    args.expected_count = args.expected_count if args.expected_count is not None else 3
    if args.expected_count < 1:
        parser.error('--expected-count must be positive')
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        parser.error('Use run_multi_button_demo.sh in the isolated mock demo')
    # Keep the context alive during Ctrl+C so action cancellation and scene
    # cleanup can finish before shutdown in the finally block.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('multi_button_task')
    report = RunReport(node, args)
    original_matrix = None
    changed_caps = set()
    inventory = {}
    active = None
    try:
        hardware = call(node, ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request())
        if not hardware.component or any(c.plugin_name != 'mock_components/GenericSystem' for c in hardware.component):
            raise RuntimeError('Execution and verification currently support mock_components/GenericSystem only')
        snapshot = scene(node)
        initial = copy.deepcopy(snapshot.robot_state)
        initial.is_diff = False
        joints = dict(zip(initial.joint_state.name, initial.joint_state.position))
        if not all(name in joints for name in ARM_JOINTS) or abs(joints.get('gripper', 1.)) > .001:
            raise RuntimeError('Expected a six-joint Piper with its gripper closed')
        report.data['initial_joints'] = {name: joints[name] for name in ARM_JOINTS}
        report.event('Initial pose saved; acquiring all targets')
        camera = None
        if args.source == 'synthetic':
            report.path.parent.mkdir(parents=True, exist_ok=True)
            (ROOT/'outputs').mkdir(exist_ok=True)
            inventory = acquire_synthetic(node)
        else:
            from camera_buttons import CameraButtons
            camera = CameraButtons(node, yaml.safe_load(Path(args.config).read_text()), calibration=args.calibration)
            inventory = camera.wait(args.expected_count)
        if len(inventory) != args.expected_count:
            raise RuntimeError(f'Expected {args.expected_count} buttons, detected {len(inventory)}')
        ordered = sorted(inventory, key=lambda key: (-inventory[key]['position'][1], inventory[key]['position'][2]))
        for key in ordered:
            target = inventory[key]
            report.data['buttons'][key] = dict(position=target['position'].tolist(), direction=target['direction'].tolist(),
                confidence=float(target['confidence']), status='pending', verification=None)
        report.data['order'] = ordered
        report.event('Inventory locked; order is descending base-frame Y')
        original_matrix = copy.deepcopy(snapshot.allowed_collision_matrix)
        update_scene(node, inventory, snapshot)
        plans = {}
        start = initial
        # Preflight every button and final return before the first execution.
        for key in ordered:
            target = inventory[key]
            apply_matrix(node, contact_matrix(original_matrix, object_id(key, 'cap')))
            approach = target_pose(target, -TIP_Z-.02)
            touch_pose = target_pose(target, -TIP_Z)
            pressed_pose = target_pose(target, -TIP_Z+args.press_depth)
            display = plan(node, approach, link_name='link6', position_tolerance=.0001, orientation_tolerance=.001, start_state=start)
            trajectories = list(display.trajectory)
            current = final_state(start, trajectories[-1])
            for destination in (touch_pose, pressed_pose, approach):
                trajectories.append(cartesian(node, current, destination))
                current = final_state(current, trajectories[-1])
            plans[key] = trajectories
            start = current
            report.event('All four motion segments planned', key)
        apply_matrix(node, original_matrix)
        returning = return_plan(node, start, initial)
        check_joints(final_state(start, returning), initial, tolerance=.0005)
        report.event('Complete itinerary and final return planned')
        if not args.execute:
            display = DisplayTrajectory(model_id='agx_arm', trajectory_start=initial)
            display.trajectory = [trajectory for key in ordered for trajectory in plans[key]] + [returning]
            pub = node.create_publisher(DisplayTrajectory, '/display_planned_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            pub.publish(display)
            for value in report.data['buttons'].values():
                value['status'] = 'planned'
            report.data['result'] = 'planned_only'
            report.event('Preview only; no presses verified or executed. Preview omits the dwell.')
            if not args.once:
                print('Enable Planned Path → Show Robot Visual in RViz. Ctrl+C closes preview.', flush=True)
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    pass
            return
        check_joints(scene(node).robot_state, initial)
        for key in ordered:
            active = key
            target = inventory[key]
            if camera:
                camera.revalidate(key, target, inventory)
            apply_matrix(node, contact_matrix(original_matrix, object_id(key, 'cap')))
            report.data['buttons'][key]['status'] = 'in_progress'
            report.event('Approach', key)
            execute(node, plans[key][0])
            if camera:
                camera.revalidate(key, target, inventory)
            report.event('Touch', key)
            execute(node, plans[key][1])
            report.event('Push', key)
            execute(node, plans[key][2])
            changed_caps.add(key)
            move_cap(node, key, target, args.press_depth)
            verify_pressed(node, key, target, args.press_depth)
            report.event(f'Hold {args.hold:.1f} seconds', key)
            began = time.monotonic()
            while time.monotonic()-began < args.hold:
                rclpy.spin_once(node, timeout_sec=max(0., min(.05, args.hold-(time.monotonic()-began))))
            elapsed = time.monotonic()-began
            report.event('Verify pressed pose and virtual switch geometry', key)
            verification = verify_pressed(node, key, target, args.press_depth)
            verification['hold_seconds'] = elapsed
            report.data['buttons'][key]['verification'] = verification
            report.event('Retract', key)
            execute(node, plans[key][3])
            move_cap(node, key, target, 0.)
            changed_caps.remove(key)
            apply_matrix(node, original_matrix)
            report.data['buttons'][key]['status'] = 'completed'
            report.event('Completed', key)
            active = None
        report.event('Return to the original pose saved before detecting buttons')
        execute(node, returning)
        error = check_joints(scene(node).robot_state, initial)
        report.data.update(result='completed', initial_pose_restored=True, return_max_joint_error_rad=error)
        report.event(f'Done: {len(ordered)}/{len(ordered)} verified in mock hardware; initial pose restored')
        if args.keep_open:
            print('Finished. RViz status remains available; motion will not repeat. Ctrl+C closes.', flush=True)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
    except (Exception, KeyboardInterrupt) as exc:
        if active:
            report.data['buttons'][active]['status'] = 'failed'
        report.data.update(result='aborted', error=str(exc) or 'Interrupted')
        report.event('Aborted; uncompleted targets remain uncompleted. No unplanned recovery motion.')
        raise
    finally:
        if rclpy.ok():
            try:
                for key in changed_caps:
                    move_cap(node, key, inventory[key], 0.)
                if original_matrix is not None:
                    apply_matrix(node, original_matrix)
            except Exception as exc:
                report.data['cleanup_error'] = str(exc)
                report.data['result'] = 'cleanup_failed'
        report.save()
        print(f'Run report: {report.path}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'ERROR: {exc}', flush=True)
        raise SystemExit(1)
