import copy
import importlib.util
from pathlib import Path
import tempfile
import subprocess
import sys

import cv2
import numpy as np
import pytest
import yaml
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray

from spectra.calibration import Calibration
from ros_test import RosTest
from test_vision import gate_image

spec = importlib.util.spec_from_file_location('calibrate_camera', Path(__file__).parents[1] / 'scripts/calibrate_camera.py')
offline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(offline)


def calibration_data():
    # Deliberately off-center: a target at x=320 is LEFT of this optical axis.
    k = [400.0, 0.0, 400.0, 0.0, 410.0, 230.0, 0.0, 0.0, 1.0]
    return {'image_width': 640, 'image_height': 480, 'camera_name': 'synthetic_test_only',
            'distortion_model': 'plumb_bob',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': k},
            'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0.1, -0.03, 0.001, 0.002, 0.0]},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': [400.0, 0.0, 400.0, 0.0, 0.0, 410.0, 230.0, 0.0, 0.0, 0.0, 1.0, 0.0]}}


def test_calibration_rectifies_once_and_rejects_mismatch(tmp_path):
    path = tmp_path / 'camera.yaml'
    data = calibration_data()
    path.write_text(yaml.safe_dump(data))
    raw = Calibration(path, 640, 480, 'raw')
    rectified = Calibration(path, 640, 480, 'rectified')
    image = gate_image(450)
    expected = cv2.undistort(image, raw.k, raw.d, None, raw.p[:, :3])
    assert np.max(np.abs(raw.rectify(image).astype(int) - expected.astype(int))) <= 1
    assert rectified.rectify(image) is image
    with pytest.raises(ValueError, match='dimensions'):
        Calibration(path, 320, 240, 'raw')
    for key, value in [('camera_matrix', float('nan')), ('projection_matrix', 0.0)]:
        invalid = copy.deepcopy(data)
        invalid[key]['data'][0] = value
        path.write_text(yaml.safe_dump(invalid))
        with pytest.raises(ValueError):
            Calibration(path, 640, 480, 'raw')
    data['distortion_model'] = 'equidistant'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='plumb_bob'):
        Calibration(path, 640, 480, 'raw')


def test_offline_solver_recovers_known_intrinsics():
    objects = np.zeros((7 * 6, 3), np.float32)
    objects[:, :2] = np.mgrid[:7, :6].T.reshape(-1, 2) * 0.03
    k = np.array([[500., 0., 310.], [0., 510., 245.], [0., 0., 1.]])
    observations = []
    for index in range(12):
        rotation = np.array([0.1 + index * 0.02, (-1)**index * 0.2, index * 0.03])
        translation = np.array([-0.1 + index * 0.005, -0.08, 0.6 + index * 0.03])
        pixels, _ = cv2.projectPoints(objects, rotation, translation, k, np.zeros(5))
        observations.append(pixels)
    result = offline.solve([objects]*12, observations, (640, 480), list(range(12)), 'fixture')
    assert np.allclose(np.array(result['camera_matrix']['data']).reshape(3, 3), k, atol=0.1)
    assert result['reprojection']['rms_pixels'] < 0.001
    assert len(result['reprojection']['views']) == 12
    assert max(v['rms_pixels'] for v in result['reprojection']['views']) < 0.001
    with pytest.raises(ValueError, match='10 usable'):
        offline.solve([], [], (640, 480), [], 'fixture')


def test_offline_rejects_mixed_sizes_and_missing_corners(tmp_path):
    a, b = tmp_path / 'a.png', tmp_path / 'b.png'
    cv2.imwrite(str(a), np.zeros((480, 640), np.uint8))
    cv2.imwrite(str(b), np.zeros((240, 320), np.uint8))
    with pytest.raises(ValueError, match='mixed image resolutions'):
        offline.calibrate([a, b], 7, 6, 0.03, 'fixture')
    with pytest.raises(ValueError, match='10 usable'):
        offline.calibrate([a], 7, 6, 0.03, 'fixture')


def test_chessboard_cli_writes_loadable_yaml_and_preserves_existing_file(tmp_path):
    board = np.kron((np.indices((7, 8)).sum(axis=0) % 2).astype(np.uint8), np.ones((40, 40), np.uint8)) * 255
    k = np.array([[500., 0., 310.], [0., 510., 245.], [0., 0., 1.]])
    source = np.float32([[0, 0], [320, 0], [320, 280], [0, 280]])
    corners = np.float32([[0, 0, 0], [0.24, 0, 0], [0.24, 0.21, 0], [0, 0.21, 0]])
    paths = []
    for index in range(12):
        destination, _ = cv2.projectPoints(corners, np.array([index * 0.02, (-1)**index * 0.2, index * 0.02]),
                                          np.array([-0.12, -0.1, 0.65 + index * 0.01]), k, np.zeros(5))
        transform = cv2.getPerspectiveTransform(source, destination.reshape(4, 2))
        image = cv2.warpPerspective(board, transform, (640, 480), borderValue=127)
        path = tmp_path / f'{index:02}.png'
        cv2.imwrite(str(path), image)
        paths.append(str(path))
    output = tmp_path / 'camera.yaml'
    command = [sys.executable, spec.origin, *paths, '--columns', '7', '--rows', '6',
               '--square-size', '0.03', '--camera-name', 'synthetic', '--output', str(output)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    original = output.read_bytes()
    assert len(yaml.safe_load(original)['reprojection']['views']) >= 10
    Calibration(output, 640, 480, 'raw')
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert result.returncode != 0
    assert output.read_bytes() == original


class CalibratedVisionTest(RosTest):
    def test_principal_point_bearing_headers_and_pair_loss(self):
        self.check_pair_fault('missing')

    def test_wrong_resolution_cannot_keep_guidance_alive(self):
        self.check_pair_fault('resolution')

    def test_wrong_timestamp_cannot_keep_guidance_alive(self):
        self.check_pair_fault('stamp')

    def test_invalid_intrinsics_cannot_keep_guidance_alive(self):
        self.check_pair_fault('intrinsics')

    def check_pair_fault(self, fault):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / 'camera.yaml'
        path.write_text(yaml.safe_dump(calibration_data()))
        self.start('helm')
        self.start('wayfinder', ('use_camera_info', 'true'))
        self.start('spectra', ('calibration_file', str(path)), ('input_geometry', 'rectified'))
        infos = self.observe('/perception/camera_info', CameraInfo, qos_profile_sensor_data)
        detections = self.observe('/perception/detections', Detection2DArray, qos_profile_sensor_data)
        health = self.observe('/vehicle/health', DiagnosticArray)
        output = self.observe('/vehicle/applied_cmd', Twist)
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        frame = [gate_image(320)]
        bridge = CvBridge()

        def publish():
            msg = bridge.cv2_to_imgmsg(frame[0], encoding='bgr8')
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            camera.publish(msg)

        timer = self.node.create_timer(0.05, publish)
        self.wait(lambda: infos and detections and health)
        self.assertEqual(infos[-1].header, detections[-1].header)
        self.assertEqual(list(infos[-1].d), [0.0]*5)
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        initial_images = len(detections)
        self.wait_armed_delivery(health)
        self.wait(lambda: len(detections) >= initial_images + 5)
        handle = self.goal('PASS_GATE')
        self.assertTrue(handle.accepted)
        self.wait(lambda: output and 0.18 < output[-1].angular.z < 0.22)
        self.assertEqual(output[-1].linear.x, 0.0)
        frame[0] = gate_image(480)
        self.wait(lambda: -0.22 < output[-1].angular.z < -0.18)
        frame[0] = gate_image(400)
        self.wait(lambda: output[-1].linear.x > 0 and abs(output[-1].angular.z) < 0.01)
        # Continue fresh detections, but no new paired CameraInfo: must fail closed.
        self.node.destroy_timer(timer)
        publisher = self.node.create_publisher(Detection2DArray, '/perception/detections', qos_profile_sensor_data)
        info_pub = self.node.create_publisher(CameraInfo, '/perception/camera_info', qos_profile_sensor_data)
        def unpaired():
            msg = copy.deepcopy(detections[-1])
            msg.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(msg)
            if fault != 'missing':
                info = copy.deepcopy(infos[-1])
                info.header.stamp = msg.header.stamp
                if fault == 'resolution':
                    info.width = 320
                elif fault == 'intrinsics':
                    info.p[0] = float('nan')
                else:
                    info.header.stamp.sec -= 1
                info_pub.publish(info)
        self.node.create_timer(0.05, unpaired)
        result = handle.get_result_async()
        self.wait(result.done, timeout=2)
        self.assertIn('camera stale', result.result().result.reason)
        self.wait(lambda: output[-1].linear.x == 0 and output[-1].angular.z == 0)
