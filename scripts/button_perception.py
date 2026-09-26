"""Marker-free RGB-D button geometry and persistent target association (no ROS)."""
from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


@dataclass
class Detection:
    position: np.ndarray
    direction: np.ndarray  # Inward pressing direction, in base_link.
    confidence: float
    diameter: float
    pixel: tuple


DEFAULTS = dict(min_area=80, min_circularity=0.60, min_yellow_fraction=0.12,
                min_diameter=0.035, max_diameter=0.070,
                min_depth=0.20, max_depth=1.20, plane_threshold=0.003,
                min_plane_inliers=0.70, max_cap_spread=0.003)


def orientation(direction):
    """Choose a repeatable tool roll; local +Z is the pressing direction."""
    z = np.asarray(direction, float)
    z /= np.linalg.norm(z)
    reference = np.array([0., 0., 1.])
    if abs(np.dot(reference, z)) > .95:
        reference = np.array([0., 1., 0.])
    # Match the existing demo's Ry(pi/2) for pressing along base +X.
    x = -reference + np.dot(reference, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()


def fit_plane(points, threshold, rng):
    if len(points) < 60:
        return None
    points = points[rng.choice(len(points), min(2000, len(points)), replace=False)]
    best = np.zeros(len(points), dtype=bool)
    for _ in range(80):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-8:
            continue
        n /= length
        selected = np.abs((points-a) @ n) < threshold
        if selected.sum() > best.sum():
            best = selected
    if best.sum() < 60:
        return None
    cloud = points[best]
    center = cloud.mean(axis=0)
    _, _, vectors = np.linalg.svd(cloud-center, full_matrices=False)
    normal = vectors[-1]
    if np.dot(normal, center) < 0:
        normal = -normal
    return normal, float(best.mean()), center


def detect_buttons(bgr, depth_m, camera_matrix, distortion, camera_to_base, settings=None):
    """Return candidates on a visible planar panel; use aligned metric depth.

    Uses the cap's own valid depth, never fills holes into synthetic measurements.
    Assumes red caps, yellow surrounds, known approximate cap size, and a locally
    planar mounting surface. It is not a universal semantic E-stop classifier.
    """
    cfg = {**DEFAULTS, **(settings or {})}
    if bgr.shape[:2] != depth_m.shape or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('RGB and aligned depth must have matching dimensions')
    k = np.asarray(camera_matrix, float).reshape(3, 3)
    transform = np.asarray(camera_to_base, float).reshape(4, 4)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    if (not np.isfinite(transform).all() or
        not np.allclose(transform[3], [0, 0, 0, 1]) or
        not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5) or
        not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Camera-to-base transform must be rigid')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    yellow = cv2.inRange(hsv, (15, 85, 70), (40, 255, 255)) > 0
    valid = np.isfinite(depth_m) & (depth_m >= cfg['min_depth']) & (depth_m <= cfg['max_depth'])
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yy, xx = np.indices(depth_m.shape)
    rng = np.random.default_rng(42)
    results = []

    def points(mask):
        v, u = np.nonzero(mask & valid)
        if not len(u):
            return np.empty((0, 3))
        rays = cv2.undistortPoints(np.column_stack([u, v]).astype(float).reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
        return np.column_stack([rays, np.ones(len(rays))]) * depth_m[v, u, None]

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * math.pi * area / max(perimeter**2, 1)
        if area < cfg['min_area'] or circularity < cfg['min_circularity'] or len(contour) < 5:
            continue
        (u, v), (a, b), _ = cv2.fitEllipse(contour)
        radius = max(a, b) / 2
        if radius < 4 or min(a, b) / max(a, b) < .6:
            continue
        distance2 = (xx-u)**2 + (yy-v)**2
        ring = (distance2 > (radius*1.1)**2) & (distance2 < (radius*1.8)**2)
        yellow_fraction = float(yellow[ring].mean()) if ring.any() else 0
        if yellow_fraction < cfg['min_yellow_fraction']:
            continue
        mask = np.zeros(depth_m.shape, np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
        cap = points(mask)
        if len(cap) < 30 or len(cap)/max(mask.sum(), 1) < .6:
            continue
        panel_mask = (distance2 > (radius*2.2)**2) & (distance2 < (radius*3.5)**2) & (red == 0) & ~yellow
        plane = fit_plane(points(panel_mask), cfg['plane_threshold'], rng)
        if plane is None:
            continue
        normal, inliers, panel_center = plane
        if inliers < cfg['min_plane_inliers'] or normal[2] < .6:
            continue
        offsets = cap @ normal
        offset = float(np.median(offsets))
        spread = float(np.median(np.abs(offsets-offset)))
        protrusion = float(panel_center @ normal - offset)
        if spread > cfg['max_cap_spread'] or not .003 < protrusion < .10:
            continue
        ray = cv2.undistortPoints(np.array([[[u, v]]]), k, distortion).reshape(2)
        ray = np.r_[ray, 1.]
        center = ray * (offset / np.dot(normal, ray))
        diameter = 2 * radius * center[2] / math.sqrt(k[0, 0]*k[1, 1])
        if not cfg['min_diameter'] < diameter < cfg['max_diameter']:
            continue
        base_center = transform[:3, :3] @ center + transform[:3, 3]
        direction = transform[:3, :3] @ normal
        confidence = min(1., circularity) * inliers * min(1., len(cap)/max(mask.sum(), 1))
        results.append(Detection(base_center, direction, confidence, diameter, (u, v)))
    return results


class Tracker:
    """One-to-one 3D association; IDs survive occlusion and are never recycled."""
    def __init__(self, min_hits=5, gate=.025, stable_radius=.003):
        self.min_hits, self.gate, self.stable_radius = min_hits, gate, stable_radius
        self.tracks = {}
        self.next_id = 1

    def update(self, detections, now):
        keys = list(self.tracks)
        matched = set()
        seen_keys = set()
        if keys and detections:
            distances = np.array([[np.linalg.norm(self.tracks[k]['position']-d.position) for d in detections] for k in keys])
            rows, cols = linear_sum_assignment(distances)
            for row, col in zip(rows, cols):
                if distances[row, col] > self.gate:
                    continue
                key, d = keys[row], detections[col]
                track = self.tracks[key]
                stable = np.linalg.norm(track['position']-d.position) <= self.stable_radius and np.dot(track['direction'], d.direction) > .995
                track['hits'] = track['hits'] + 1 if stable and track['visible'] else 1
                track.update(position=d.position.copy(), direction=d.direction.copy(), confidence=d.confidence, last_seen=now, visible=True)
                matched.add(col)
                seen_keys.add(key)
        for key in keys:
            if key not in seen_keys:
                self.tracks[key]['visible'] = False
                self.tracks[key]['hits'] = 0
        for index, d in enumerate(detections):
            if index in matched:
                continue
            key = f'button_{self.next_id:03d}'
            self.next_id += 1
            self.tracks[key] = dict(position=d.position.copy(), direction=d.direction.copy(), confidence=d.confidence, last_seen=now, visible=True, hits=1)

    def ready(self, now, max_age=.5):
        return {key: dict(value) for key, value in self.tracks.items()
                if value['visible'] and value['hits'] >= self.min_hits and now-value['last_seen'] <= max_age}


def synthetic_rgbd():
    """Explicit synthetic test image, not camera acquisition or Gazebo rendering."""
    image = np.full((480, 640, 3), 125, np.uint8)
    depth = np.full((480, 640), .64, np.float32)
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    transform = np.array([[0., 0, 1, -.1], [-1., 0, 0, 0], [0., -1, 0, .3], [0., 0, 0, 1]])
    for u in (200, 320, 440):
        cv2.rectangle(image, (u-45, 195), (u+45, 285), (0, 220, 240), -1)
        depth[195:286, u-45:u+46] = .61
        cv2.circle(image, (u, 240), 25, (0, 0, 220), -1)
        yy, xx = np.indices(depth.shape)
        depth[(xx-u)**2+(yy-240)**2 <= 25**2] = .60
    return image, depth, k, np.zeros(5), transform
