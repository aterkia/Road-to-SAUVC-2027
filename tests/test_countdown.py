"""Countdown is one-shot and cannot be bypassed by a start service."""

import subprocess
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from ros_test import RosTest


class CountdownTest(RosTest):
    def test_unverified_hardware_config_is_rejected_before_nodes_start(self):
        result = subprocess.run(['ros2', 'launch', 'bootstrap', 'auv.launch.py', 'mode:=hardware'],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Hardware requires verified firmware', result.stdout + result.stderr)
        self.assertNotIn('process started with pid', result.stdout + result.stderr)

    def start_countdown(self, delay=0.8):
        self.status = self.observe('/mission/status', String,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.arm_requests = []

        def arm(request, response):
            self.arm_requests.append(request.data)
            response.success = not request.data
            return response

        self.arm_service = self.node.create_service(SetBool, '/vehicle/arm', arm)
        self.health = self.node.create_publisher(DiagnosticArray, '/vehicle/health', 1)
        self.ready = False

        def publish():
            message = DiagnosticArray()
            message.header.stamp = self.node.get_clock().now().to_msg()
            status = DiagnosticStatus(name='helm', level=DiagnosticStatus.WARN, message='DISARMED')
            status.values = [KeyValue(key='preflight_ready', value=str(self.ready).lower())]
            message.status = [status]
            self.health.publish(message)

        self.timer = self.node.create_timer(0.05, publish)
        self.start('orchestrator', ('vehicle_mode', 'hardware'),
                   ('tasks', '[DIVE_TO_DEPTH, TURN]'), ('start_delay_s', delay))
        self.wait(lambda: any(s.data.startswith('COUNTDOWN') for s in self.status))

    def test_failed_preflight_never_arms_and_later_recovery_does_not_restart(self):
        self.start_countdown()
        self.assertFalse(self.call('/mission/start', Trigger, Trigger.Request()).success)
        self.wait(lambda: any(s.data.startswith('FAILED: disarm confirmed') for s in self.status))
        self.assertNotIn(True, self.arm_requests)
        self.ready = True
        mark = len(self.status)
        ticks = []
        timer = self.node.create_timer(0.1, lambda: ticks.append(True))
        self.wait(lambda: len(ticks) >= 12)
        self.node.destroy_timer(timer)
        self.assertFalse(any(s.data.startswith(('COUNTDOWN', 'RUNNING')) for s in self.status[mark:]))
        self.assertNotIn(True, self.arm_requests)

    def test_abort_during_countdown_cancels_start(self):
        self.start_countdown(delay=2.0)
        self.assertTrue(self.call('/mission/abort', Trigger, Trigger.Request()).success)
        self.wait(lambda: any(s.data == 'ABORTED: disarm confirmed' for s in self.status))
        self.assertNotIn(True, self.arm_requests)
