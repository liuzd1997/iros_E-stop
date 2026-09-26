#!/usr/bin/env python3
"""Inspect marker-free D435i detections; never command the robot."""
import argparse
import json
import time
from pathlib import Path
import rclpy
import yaml
from camera_buttons import CameraButtons

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'config/d435i_buttons.yaml'))
parser.add_argument('--camera-frame', action='store_true', help='Inspect before extrinsic calibration; coordinates are NOT robot targets')
parser.add_argument('--calibration', help='Measured fixed-camera transform YAML; otherwise use tf2')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('inspect_camera_buttons')
try:
    detector = CameraButtons(node, yaml.safe_load(Path(args.config).read_text()), camera_frame_only=args.camera_frame, calibration=args.calibration)
    print('Detection only. No robot motion. Ctrl+C stops.', flush=True)
    last = 0.
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)
        if time.monotonic()-last > 2:
            last = time.monotonic()
            targets = {key: dict(position=value['position'].tolist(), direction=value['direction'].tolist()) for key, value in detector.ready().items()}
            print(json.dumps(dict(frame=detector.frame, targets=targets, status=detector.last_error)), flush=True)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
