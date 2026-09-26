#!/usr/bin/env python3
"""Fit a fixed optical-camera-to-base transform from measured point pairs (metres)."""
import argparse
from pathlib import Path
import numpy as np
import yaml


def fit_transform(camera, base, max_error=.005):
    camera, base = np.asarray(camera, float), np.asarray(base, float)
    if camera.shape != base.shape or camera.ndim != 2 or camera.shape[1] != 3 or len(camera) < 6:
        raise ValueError('Provide at least six corresponding 3D points in each frame')
    if not np.isfinite(camera).all() or not np.isfinite(base).all():
        raise ValueError('All measured points must be finite')
    a, b = camera-camera.mean(axis=0), base-base.mean(axis=0)
    for cloud in (a, b):
        if np.linalg.svd(cloud, compute_uv=False)[1] < .03:
            raise ValueError('Points must span two dimensions by at least 30 mm; avoid clustered/collinear points')
    u, _, vt = np.linalg.svd(a.T @ b)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = base.mean(axis=0)-rotation @ camera.mean(axis=0)
    error = np.linalg.norm(camera @ rotation.T+translation-base, axis=1)
    if error.max() > max_error:
        raise ValueError(f'Calibration residual {error.max()*1000:.2f} mm exceeds {max_error*1000:.2f} mm')
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    return transform, error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements', help='YAML containing camera_frame, camera_points, base_points')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    data = yaml.safe_load(Path(args.measurements).read_text())
    if not data.get('camera_frame'):
        parser.error('Measurements must name the optical camera frame')
    matrix, errors = fit_transform(data['camera_points'], data['base_points'])
    result = dict(calibrated=True, parent_frame='base_link', camera_frame=data['camera_frame'],
                  camera_to_base=matrix.tolist(), sample_count=len(errors),
                  rms_error_m=float(np.sqrt(np.mean(errors**2))), max_error_m=float(errors.max()))
    Path(args.output).write_text(yaml.safe_dump(result, sort_keys=False))
    print(f'Saved {args.output}: RMS {result["rms_error_m"]*1000:.2f} mm. Validate against independent measured points before use.')


if __name__ == '__main__':
    main()
