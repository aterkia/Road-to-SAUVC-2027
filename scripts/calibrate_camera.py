#!/usr/bin/env python3
"""Offline chessboard calibration. Outputs ROS camera YAML plus per-view RMS."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def solve(object_points, image_points, size, names, camera_name):
    if len(image_points) < 10:
        raise ValueError('at least 10 usable views with varied position/orientation are required')
    rms, k, d, rotations, translations = cv2.calibrateCamera(
        object_points, image_points, size, None, None
    )
    if not math.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(d).all():
        raise ValueError('calibration did not produce finite parameters')

    # Per-view residuals help reveal bad views hidden by the aggregate RMS.
    errors = []
    for name, objects, observed, rotation, translation in zip(
        names, object_points, image_points, rotations, translations
    ):
        projected, _ = cv2.projectPoints(objects, rotation, translation, k, d)
        error = np.sqrt(
            np.mean(np.sum((observed.reshape(-1, 2) - projected.reshape(-1, 2)) ** 2, axis=1))
        )
        errors.append({'image': str(name), 'rms_pixels': float(error)})

    def matrix(value):
        return {'rows': value.shape[0], 'cols': value.shape[1], 'data': value.ravel().tolist()}

    # Keep the full image and original principal point; no ROI/crop adjustment.
    return {
        'image_width': size[0],
        'image_height': size[1],
        'camera_name': camera_name,
        'distortion_model': 'plumb_bob',
        'camera_matrix': matrix(k),
        'distortion_coefficients': matrix(d.reshape(1, 5)),
        'rectification_matrix': matrix(np.eye(3)),
        'projection_matrix': matrix(np.column_stack((k, np.zeros(3)))),
        'reprojection': {'rms_pixels': float(rms), 'views': errors},
    }


def calibrate(paths, columns, rows, square_size, camera_name):
    if columns < 3 or rows < 3 or not math.isfinite(square_size) or square_size <= 0:
        raise ValueError('need at least 3x3 inner corners and a positive square size in metres')
    objects = np.zeros((rows * columns, 3), np.float32)
    objects[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2) * square_size

    image_points, names, rejected = [], [], []
    size = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'cannot read {path}')
        current_size = (image.shape[1], image.shape[0])
        if size is not None and current_size != size:
            raise ValueError(f'mixed image resolutions: {path}')
        size = current_size
        found, corners = cv2.findChessboardCorners(image, (columns, rows))
        if not found:
            rejected.append(str(path))
            continue
        refined = cv2.cornerSubPix(
            image,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        image_points.append(refined)
        names.append(path)

    result = solve([objects] * len(names), image_points, size, names, camera_name)
    result['reprojection']['rejected_images'] = rejected
    result['reprojection']['board'] = {
        'columns': columns,
        'rows': rows,
        'square_size_m': square_size,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', nargs='+', type=Path)
    parser.add_argument('--columns', required=True, type=int, help='inner corners horizontally')
    parser.add_argument('--rows', required=True, type=int, help='inner corners vertically')
    parser.add_argument('--square-size', required=True, type=float, help='metres')
    parser.add_argument('--camera-name', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    try:
        result = calibrate(
            sorted(set(args.images)), args.columns, args.rows, args.square_size, args.camera_name
        )
        # An existing measured calibration is never silently overwritten.
        with args.output.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(result, stream, sort_keys=False)
    except (ValueError, OSError, cv2.error) as error:
        parser.exit(1, f'Calibration failed: {error}\n')
    print(f"RMS: {result['reprojection']['rms_pixels']:.4f} px; review every view in {args.output}")


if __name__ == '__main__':
    main()
