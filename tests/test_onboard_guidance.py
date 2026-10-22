"""Hardware guidance consumes stamped depth/IMU, never fabricated XY odometry."""

import math

from aterkia_interfaces.msg import DepthStamped, VehicleCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from ros_test import RosTest


class OnboardGuidanceTest(RosTest):
    def fixture(self, depth=0.0, visual=False):
        self.depth, self.yaw, self.freeze_depth = depth, 3.0, False
        self.armed = False
        self.frozen_stamp = None
        self.commands = self.observe('/vehicle/command', VehicleCommand)
        telemetry = self.sensor_node('guidance_sensor_fixture')
        self.imu = telemetry.create_publisher(Imu, '/vehicle/imu', qos_profile_sensor_data)
        self.depth_pub = telemetry.create_publisher(DepthStamped, '/vehicle/depth_stamped', qos_profile_sensor_data)
        self.health = telemetry.create_publisher(DiagnosticArray, '/vehicle/health', 1)

        def sensors():
            stamp = self.node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
            imu.orientation.w = math.cos(self.yaw/2)
            imu.orientation.z = math.sin(self.yaw/2)
            self.imu.publish(imu)
            depth = DepthStamped(depth_m=self.depth)
            depth.header = imu.header
            if self.freeze_depth:
                depth.header.stamp = self.frozen_stamp
            else:
                self.frozen_stamp = stamp
            self.depth_pub.publish(depth)
            health = DiagnosticArray()
            health.header.stamp = stamp
            health.status = [DiagnosticStatus(name='helm',
                level=DiagnosticStatus.OK if self.armed else DiagnosticStatus.WARN,
                message='ARMED' if self.armed else 'DISARMED',
                values=[KeyValue(key='preflight_ready', value='false' if self.armed else 'true')])]
            self.health.publish(health)

        self.timer = telemetry.create_timer(0.05, sensors)
        if visual:
            self.camera_fixture()
        self.start('wayfinder', ('vehicle_mode', 'hardware'), ('target_depth_m', 1.0),
                   ('settle_time_s', 0.15), ('use_camera_info', str(visual).lower()),
                   ('require_camera_on_start', str(visual).lower()))
        self.wait_navigation_ready()
        self.armed = True
        self.wait(lambda: len(self.commands) >= 3)

    def camera_fixture(self):
        self.visible, self.send_image = False, True
        self.pixel, self.box_width, self.frame_budget = 400.0, 200.0, None
        self.camera_stamp = None
        detections = self.node.create_publisher(Detection2DArray, '/perception/detections', qos_profile_sensor_data)
        calibration = self.node.create_publisher(CameraInfo, '/perception/camera_info', qos_profile_sensor_data)

        def camera():
            if not self.send_image:
                return
            if self.frame_budget is None or self.frame_budget > 0:
                self.camera_stamp = self.node.get_clock().now().to_msg()
                if self.frame_budget is not None:
                    self.frame_budget -= 1
            message = Detection2DArray()
            message.header.stamp, message.header.frame_id = self.camera_stamp, 'camera_optical_frame'
            if self.visible:
                detection = Detection2D()
                detection.header = message.header
                detection.bbox.center.position.x, detection.bbox.center.position.y = self.pixel, 240.0
                detection.bbox.size_x, detection.bbox.size_y = self.box_width, 280.0
                result = ObjectHypothesisWithPose()
                result.hypothesis.class_id, result.hypothesis.score = 'gate', 0.9
                detection.results = [result]
                message.detections = [detection]
            info = CameraInfo(width=640, height=480, distortion_model='plumb_bob',
                d=[0.0]*5, k=[400., 0., 400., 0., 400., 240., 0., 0., 1.],
                r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                p=[400., 0., 400., 0., 0., 400., 240., 0., 0., 0., 1., 0.])
            info.header = message.header
            calibration.publish(info)
            detections.publish(message)

        self.image_timer = self.node.create_timer(0.05, camera)

    def test_depth_requires_measured_stable_target_and_timeout_is_failure(self):
        self.fixture()
        first = self.goal('DIVE_TO_DEPTH', timeout=0.5)
        self.assertTrue(first.accepted)
        result = first.get_result_async()
        self.wait(lambda: any(c.heave_input < 0 for c in self.commands))
        self.wait(result.done)
        self.assertEqual(result.result().result.status, 'FAILED')
        self.assertIn('timeout', result.result().result.reason)
        second = self.goal('DIVE_TO_DEPTH', timeout=2.0)
        self.assertTrue(second.accepted)
        result = second.get_result_async()
        self.depth = 1.0
        self.wait(result.done)
        self.assertEqual(result.result().result.status, 'COMPLETED')
        self.assertEqual(self.commands[-1].heave_input, 0.0)

    def test_turn_wrap_uses_imu_and_preserves_depth(self):
        self.fixture(depth=1.0)
        handle = self.goal('TURN', turn_radians=0.4)
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.wait(lambda: self.commands[-1].yaw_input > 0)
        self.yaw = 3.4 - 2*math.pi
        self.wait(result.done)
        self.assertEqual(result.result().result.status, 'COMPLETED')
        self.assertEqual(self.commands[-1].yaw_input, 0.0)

    def test_replayed_depth_fails_even_with_live_imu_and_health(self):
        self.fixture()
        handle = self.goal('DIVE_TO_DEPTH')
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.freeze_depth = True
        self.wait(result.done, timeout=2)
        self.assertEqual(result.result().result.status, 'FAILED')
        self.assertIn('telemetry stale', result.result().result.reason)
        self.wait(lambda: self.commands[-1].heave_input == 0.0, timeout=1)
        self.assertEqual(self.commands[-1].heave_input, 0.0)

    def test_hardware_pass_gate_remains_unavailable(self):
        self.fixture()
        self.assertFalse(self.goal('PASS_GATE').accepted)

    def test_visual_search_align_and_approach_stop_at_near_threshold(self):
        self.fixture(depth=1.0, visual=True)
        goal = self.goal('APPROACH_GATE')
        self.assertTrue(goal.accepted)
        result = goal.get_result_async()
        self.wait(lambda: self.commands[-1].yaw_input > 0)
        self.visible, self.pixel = True, 450.0
        self.wait(lambda: self.commands[-1].yaw_input < 0)
        self.assertEqual(self.commands[-1].surge_input, 0.0)
        self.pixel = 400.0
        self.wait(lambda: self.commands[-1].surge_input > 0)
        self.box_width = 400.0
        self.wait(result.done)
        self.assertEqual(result.result().result.status, 'COMPLETED')
        self.assertIn('passage not claimed', result.result().result.reason)
        self.assertEqual(self.commands[-1].surge_input, 0.0)

    def test_camera_loss_during_approach_fails_and_stops(self):
        self.fixture(depth=1.0, visual=True)
        self.visible = True
        goal = self.goal('APPROACH_GATE')
        self.assertTrue(goal.accepted)
        result = goal.get_result_async()
        self.wait(lambda: self.commands[-1].surge_input > 0)
        self.send_image = False
        self.wait(result.done, timeout=2)
        self.assertEqual(result.result().result.status, 'FAILED')
        self.assertIn('camera stale', result.result().result.reason)
        self.wait(lambda: self.commands[-1].surge_input == 0.0, timeout=1)
        self.assertEqual(self.commands[-1].surge_input, 0.0)

    def test_one_near_image_replayed_cannot_complete_approach(self):
        self.fixture(depth=1.0, visual=True)
        goal = self.goal('APPROACH_GATE')
        self.assertTrue(goal.accepted)
        result = goal.get_result_async()
        self.visible, self.box_width, self.frame_budget = True, 400.0, 1
        self.wait(result.done, timeout=2)
        self.assertEqual(result.result().result.status, 'FAILED')
        self.assertIn('camera stale', result.result().result.reason)
