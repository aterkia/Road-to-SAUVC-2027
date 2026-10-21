"""Guidance receives synthetic sensor messages but commands cannot move the fixture."""

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from ros_test import RosTest


class GuidanceFaultTest(RosTest):
    def setUp(self):
        super().setUp()
        self.start('wayfinder')
        self.output = self.observe('/cmd_vel', Twist)
        self.status = self.observe('/navigation/status', String)
        self.odom_pub = self.node.create_publisher(
            Odometry, '/vehicle/odometry', qos_profile_sensor_data
        )
        self.health_pub = self.node.create_publisher(DiagnosticArray, '/vehicle/health', 1)
        self.camera_pub = self.node.create_publisher(
            Detection2DArray, '/perception/detections', qos_profile_sensor_data
        )
        self.show_gate = True
        self.freeze_odom = False
        self.freeze_camera = False
        self.saved_stamp = None
        self.sent = 0
        self.timer = self.node.create_timer(0.05, self.sensors)
        self.wait(
            lambda: self.output
            and self.odom_pub.get_subscription_count()
            and self.health_pub.get_subscription_count()
            and self.camera_pub.get_subscription_count()
        )
        baseline = self.sent
        self.wait(lambda: self.sent >= baseline + 3)

    def sensors(self):
        stamp = self.node.get_clock().now().to_msg()
        if self.saved_stamp is None:
            self.saved_stamp = stamp
        odom = Odometry()
        odom.header.stamp = self.saved_stamp if self.freeze_odom else stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.z = -1.0
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        health = DiagnosticArray()
        health.header.stamp = stamp
        health.status = [DiagnosticStatus(name='helm', level=DiagnosticStatus.OK, message='ARMED')]
        self.health_pub.publish(health)
        image = Detection2DArray()
        image.header.stamp = self.saved_stamp if self.freeze_camera else stamp
        image.header.frame_id = 'camera_optical_frame'
        if self.show_gate:
            detection = Detection2D()
            detection.header = image.header
            detection.bbox.center.position.x = 320.0
            detection.bbox.center.position.y = 240.0
            detection.bbox.size_x = 400.0
            detection.bbox.size_y = 300.0
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'gate'
            hypothesis.hypothesis.score = 0.95
            detection.results = [hypothesis]
            image.detections = [detection]
        self.camera_pub.publish(image)
        self.sent += 1

    def test_disappearance_and_time_without_displacement_cannot_succeed(self):
        handle = self.goal('PASS_GATE', timeout=1.0)
        self.assertTrue(handle.accepted)
        self.wait(lambda: any(m.data == 'RUNNING: PASS' for m in self.status))
        self.show_gate = False
        future = handle.get_result_async()
        self.wait(future.done, timeout=3)
        self.assertEqual(future.result().status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(future.result().result.status, 'FAILED')
        self.assertIn('timeout', future.result().result.reason)
        self.wait(lambda: self.output[-1].linear.x == 0)

    def test_repeated_old_telemetry_does_not_keep_task_alive(self):
        handle = self.goal('TURN')
        self.assertTrue(handle.accepted)
        self.freeze_odom = True
        future = handle.get_result_async()
        self.wait(future.done, timeout=2)
        self.assertEqual(future.result().result.status, 'FAILED')
        self.assertIn('telemetry', future.result().result.reason)
        self.wait(lambda: self.output[-1].angular.z == 0)

    def test_repeated_old_camera_does_not_keep_task_alive(self):
        handle = self.goal('PASS_GATE')
        self.assertTrue(handle.accepted)
        self.freeze_camera = True
        future = handle.get_result_async()
        self.wait(future.done, timeout=2)
        self.assertEqual(future.result().result.status, 'FAILED')
        self.assertIn('camera', future.result().result.reason)
        self.wait(lambda: self.output[-1].linear.x == 0)
