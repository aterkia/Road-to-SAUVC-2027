"""Fixed, full-resolution monocular calibration; no implicit scaling or cropping."""

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo


class Calibration:
    def __init__(self, path, width, height, input_geometry):
        if input_geometry not in ('raw', 'rectified'):
            raise ValueError('calibrated input_geometry must be raw or rectified')

        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if (
            not isinstance(data, dict)
            or data.get('image_width') != width
            or data.get('image_height') != height
            or data.get('distortion_model') != 'plumb_bob'
        ):
            raise ValueError('calibration must match image dimensions and use plumb_bob')

        def matrix(name, rows, cols):
            entry = data[name]
            values = np.asarray(entry['data'], dtype=np.float64)
            if entry['rows'] != rows or entry['cols'] != cols or values.size != rows * cols:
                raise ValueError(f'invalid {name} dimensions')
            if not np.isfinite(values).all():
                raise ValueError(f'nonfinite {name}')
            return values.reshape(rows, cols)

        self.k = matrix('camera_matrix', 3, 3)
        self.d = matrix('distortion_coefficients', 1, 5)
        self.r = matrix('rectification_matrix', 3, 3)
        self.p = matrix('projection_matrix', 3, 4)

        for intrinsics in (self.k, self.p[:, :3]):
            if (
                intrinsics[0, 0] <= 0
                or intrinsics[1, 1] <= 0
                or not 0 <= intrinsics[0, 2] < width
                or not 0 <= intrinsics[1, 2] < height
                or not np.array_equal(intrinsics[2], [0, 0, 1])
                or intrinsics[0, 1] != 0
                or intrinsics[1, 0] != 0
            ):
                raise ValueError('invalid pinhole intrinsics')
        if not np.array_equal(self.r, np.eye(3)) or np.any(self.p[:, 3] != 0):
            raise ValueError('only monocular calibration with identity rectification is supported')

        self.width, self.height = width, height
        self.maps = None

        # Build undistortion maps once and reuse them for every raw frame.
        if input_geometry == 'raw':
            self.maps = cv2.initUndistortRectifyMap(
                self.k, self.d, self.r, self.p[:, :3], (width, height), cv2.CV_32FC1
            )

    def rectify(self, image):
        if image.shape[:2] != (self.height, self.width):
            raise ValueError('image resolution differs from calibration')
        if self.maps is None:
            return image  # The driver already rectified using this P.
        return cv2.remap(image, *self.maps, cv2.INTER_LINEAR)

    def camera_info(self, header):
        info = CameraInfo()
        info.header = header
        info.width, info.height = self.width, self.height
        info.distortion_model = 'plumb_bob'
        # Describe the processed image, so downstream cannot undistort it again.
        info.k = self.p[:, :3].ravel().tolist()
        info.d = [0.0] * 5
        info.r = self.r.ravel().tolist()
        info.p = self.p.ravel().tolist()
        return info
