"""ROS D435i adapter for the marker-free detector. No motion commands."""
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from button_perception import Tracker, detect_buttons


def decode_image(msg):
    specifications = {'bgr8': ('u1', 3), 'rgb8': ('u1', 3), '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in specifications:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    dtype, channels = specifications[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + dtype)
    if msg.step < msg.width * channels * dtype.itemsize or len(msg.data) < msg.step * msg.height:
        raise ValueError('Image row stride/data length is inconsistent')
    shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    strides = (msg.step, channels*dtype.itemsize, dtype.itemsize) if channels > 1 else (msg.step, dtype.itemsize)
    array = np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
    if msg.encoding == 'rgb8':
        array = array[:, :, ::-1].copy()
    if msg.encoding == '16UC1':
        array = array.astype(np.float32) * .001  # ROS depth convention: millimetres.
    if msg.encoding == '32FC1':
        array = array.astype(np.float32)  # ROS depth convention: metres.
    return array


def load_calibration(path, *, allow_assumed=False):
    data = yaml.safe_load(Path(path).read_text())
    accepted = data.get('calibrated') is True or (allow_assumed and data.get('assumed') is True)
    if not accepted or data.get('parent_frame') != 'base_link':
        raise ValueError('Calibration must be measured, marked calibrated: true, and expressed in base_link')
    matrix = np.array(data['camera_to_base'], float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
        not np.allclose(matrix[3], [0, 0, 0, 1]) or
        not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) or
        not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Calibration matrix is not a rigid transform')
    if not data.get('camera_frame'):
        raise ValueError('Calibration must name the exact camera optical frame')
    return data, matrix


class CameraButtons:
    def __init__(self, node, config, *, camera_frame_only=False, calibration=None, allow_assumed=False):
        self.node = node
        self.config = config
        self.camera_frame_only = camera_frame_only
        self.calibration = load_calibration(calibration, allow_assumed=allow_assumed) if calibration else None
        self.info = None
        self.tracker = Tracker(min_hits=int(config.get('stable_frames', 5)))
        self.last_error = 'Waiting for synchronized RGB, aligned depth, and CameraInfo'
        self.frame = None
        self.last_processed = 0.
        self.last_stamp = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.info_sub = node.create_subscription(CameraInfo, config['camera_info_topic'], self.set_info, qos_profile_sensor_data)
        self.rgb = message_filters.Subscriber(node, Image, config['color_topic'], qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(node, Image, config['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb, self.depth], 5, .04)
        self.sync.registerCallback(self.callback)
        self.overlay = node.create_publisher(Image, '/button_demo/detection_image', 1)

    def set_info(self, message):
        self.info = message

    def callback(self, rgb, depth):
        now = time.monotonic()
        if now-self.last_processed < .1:
            return
        self.last_processed = now
        try:
            if self.info is None:
                raise ValueError('Waiting for camera calibration (CameraInfo)')
            if (self.info.width, self.info.height) != (rgb.width, rgb.height):
                raise ValueError('CameraInfo and RGB dimensions do not match')
            if self.info.header.frame_id != rgb.header.frame_id or depth.header.frame_id != rgb.header.frame_id:
                raise ValueError('Use depth aligned to color and matching color optical-frame CameraInfo')
            if self.info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('Unsupported distortion model')
            stamp = Time.from_msg(rgb.header.stamp)
            age = (self.node.get_clock().now()-stamp).nanoseconds / 1e9
            if age > .5 or age < -.1:
                raise ValueError('Camera image timestamp is stale or in the future')
            if self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp:
                raise ValueError('Camera timestamps are not increasing')
            self.last_stamp = stamp.nanoseconds
            if self.camera_frame_only:
                transform = np.eye(4)
                self.frame = rgb.header.frame_id
            elif self.calibration:
                data, transform = self.calibration
                if data['camera_frame'] != rgb.header.frame_id:
                    raise ValueError('Measured calibration is for a different camera optical frame')
                self.frame = 'base_link'
            else:
                tf = self.buffer.lookup_transform('base_link', rgb.header.frame_id, stamp)
                q, p = tf.transform.rotation, tf.transform.translation
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                transform[:3, 3] = [p.x, p.y, p.z]
                self.frame = 'base_link'
            bgr = decode_image(rgb)
            metric_depth = decode_image(depth)
            if metric_depth.ndim != 2 or depth.encoding not in ('16UC1', '32FC1'):
                raise ValueError('Depth must use 16UC1 or 32FC1 encoding')
            detections = detect_buttons(bgr, metric_depth, np.array(self.info.k).reshape(3, 3),
                                         np.array(self.info.d), transform, self.config.get('detector'))
            self.tracker.update(detections, now)
            self.last_error = '' if detections else 'No valid red/yellow button geometry detected'
            for d in detections:
                u, v = map(lambda x: int(round(x)), d.pixel)
                cv2.circle(bgr, (u, v), 7, (0, 255, 0), 2)
                cv2.putText(bgr, f'{d.confidence:.2f}', (u+10, v), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
            out = Image()
            out.header = rgb.header
            out.height, out.width = bgr.shape[:2]
            out.encoding = 'bgr8'
            out.step = out.width * 3
            out.data = bgr.tobytes()
            self.overlay.publish(out)
        except (ValueError, TransformException, cv2.error) as exc:
            self.last_error = str(exc)
            # A bad frame breaks the required run of stable detections.
            self.tracker.update([], now)

    def ready(self):
        return self.tracker.ready(time.monotonic())

    def wait(self, count, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=.05)
            targets = self.ready()
            if len(targets) == count:
                return targets
            if len(targets) > count:
                raise RuntimeError(f'Detected {len(targets)} eligible buttons; expected {count}. Resolve the inventory before motion.')
        raise RuntimeError(f'Could not acquire {count} stable targets: {self.last_error or "too few visible buttons"}')

    def revalidate(self, key, target, inventory):
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=.05)
            targets = self.ready()
            if set(targets)-set(inventory):
                raise RuntimeError('New/unmatched button detected; stop and acquire a new inventory')
            if key in targets:
                actual = targets[key]
                if (np.linalg.norm(actual['position']-target['position']) > .005 or
                    np.dot(actual['direction'], target['direction']) < .995):
                    raise RuntimeError(f'{key} moved or its estimated pose changed; plan is no longer valid')
                return
        raise RuntimeError(f'{key} is stale or occluded; no blind press will be attempted')
