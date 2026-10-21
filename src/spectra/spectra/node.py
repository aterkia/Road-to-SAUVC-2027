import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from spectra.detector import detect_gate, detect_finals
from spectra.calibration import Calibration


class Spectra(Node):
    def __init__(self):
        super().__init__('spectra')
        fixed = ParameterDescriptor(read_only=True)
        self.lower = self.declare_parameter('hsv_lower', [0, 120, 80], fixed).value
        self.upper = self.declare_parameter('hsv_upper', [25, 255, 255], fixed).value
        self.min_area = self.declare_parameter('min_post_area', 80.0, fixed).value
        self.min_aspect = self.declare_parameter('min_post_aspect', 2.0, fixed).value
        self.max_age = self.declare_parameter('image_timeout', 0.5, fixed).value
        self.frame = self.declare_parameter('camera_frame', 'camera_optical_frame', fixed).value
        self.width = self.declare_parameter('image_width', 640, fixed).value
        self.height = self.declare_parameter('image_height', 480, fixed).value

        # Calibration must describe the geometry of the incoming image.
        geometry = self.declare_parameter('input_geometry', 'uncalibrated', fixed).value
        calibration_file = self.declare_parameter('calibration_file', '', fixed).value
        if geometry not in ('uncalibrated', 'raw', 'rectified') or (
            bool(calibration_file) != (geometry != 'uncalibrated')
        ):
            raise ValueError(
                'raw/rectified input requires calibration_file; uncalibrated forbids it'
            )
        self.calibration = (
            Calibration(calibration_file, self.width, self.height, geometry)
            if calibration_file
            else None
        )

        self.finals = self.declare_parameter('enable_finals', False, fixed).value
        self.drum_lower = self.declare_parameter('drum_hsv_lower', [90, 120, 80], fixed).value
        self.drum_upper = self.declare_parameter('drum_hsv_upper', [130, 255, 255], fixed).value
        self.finals_area = self.declare_parameter('finals_min_area', 200.0, fixed).value
        self.flare_ranges = []
        for color, lower, upper in [
            ('red', [0, 150, 80], [8, 255, 255]),
            ('green', [40, 120, 80], [85, 255, 255]),
            ('yellow', [26, 120, 80], [38, 255, 255]),
        ]:
            lo = self.declare_parameter(f'{color}_hsv_lower', lower, fixed).value
            hi = self.declare_parameter(f'{color}_hsv_upper', upper, fixed).value
            self.flare_ranges.append((f'flare_{color}', lo, hi))
        if (
            len(self.lower) != 3
            or len(self.upper) != 3
            or any(
                not 0 <= lo <= hi <= cap
                for lo, hi, cap in zip(self.lower, self.upper, [179, 255, 255])
            )
            or any(
                not math.isfinite(v) or v <= 0
                for v in [self.min_area, self.min_aspect, self.max_age, self.finals_area]
            )
            or self.width <= 0
            or self.height <= 0
            or not self.frame
        ):
            raise ValueError('invalid detector parameters')
        for _, lo, hi in [('drum', self.drum_lower, self.drum_upper)] + self.flare_ranges:
            if (
                len(lo) != 3
                or len(hi) != 3
                or any(not 0 <= a <= b <= cap for a, b, cap in zip(lo, hi, [179, 255, 255]))
            ):
                raise ValueError('invalid finals HSV range')

        self.bridge = CvBridge()
        # Small frames do not benefit from a worker per host CPU in a container.
        cv2.setNumThreads(1)
        self.publisher = self.create_publisher(
            Detection2DArray, '/perception/detections', qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, '/perception/camera_info', qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info('Gate detector ready; calibrate HSV using pool recordings')

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (
            msg.header.frame_id != self.frame
            or not -0.05 <= age <= self.max_age
            or msg.width != self.width
            or msg.height != self.height
            or msg.encoding not in ('bgr8', 'rgb8')
            or msg.step < msg.width * 3
            or len(msg.data) != msg.step * msg.height
        ):
            self.get_logger().warning(
                'Rejected stale/invalid camera frame', throttle_duration_sec=2.0
            )
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.calibration:
                image = self.calibration.rectify(image)
            gate = detect_gate(image, self.lower, self.upper, self.min_area, self.min_aspect)
        except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f'Rejected image: {error}', throttle_duration_sec=2.0)
            return

        # Keep source headers so guidance can pair detections with CameraInfo.
        output = Detection2DArray()
        output.header = msg.header
        targets = [('gate', gate)] if gate else []
        if self.finals:
            targets += detect_finals(
                image, self.drum_lower, self.drum_upper, self.flare_ranges, self.finals_area
            )

        for label, box in targets:
            x, y, width, height, confidence = box
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = x
            detection.bbox.center.position.y = y
            detection.bbox.size_x = width
            detection.bbox.size_y = height
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)
            output.detections.append(detection)

        # Empty arrays mean a fresh image with no target; silence means camera failure.
        if self.calibration:
            self.info_publisher.publish(self.calibration.camera_info(msg.header))
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Spectra()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
