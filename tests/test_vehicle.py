import math

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

from ros_test import RosTest


class VehicleTest(RosTest):
    def setUp(self):
        super().setUp()
        self.output = self.observe('/vehicle/applied_cmd', Twist)
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.odom = self.observe('/vehicle/odometry', Odometry, qos_profile_sensor_data)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 1)
        self.start('helm')
        self.wait(lambda: self.output and self.odom and self.pub.get_subscription_count())

    def arm(self):
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.wait(lambda: self.health[-1].status[0].message == 'ARMED')

    def move(self, x=0.3, yaw=0.2):
        self.arm()
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = yaw
        self.pub.publish(msg)
        self.wait(lambda: self.output[-1].linear.x > 0)

    def test_startup_unarmed_limits_frames_watchdog_and_no_resume(self):
        command = Twist(); command.linear.x = 4.0
        self.pub.publish(command)
        self.wait(lambda: len(self.output) > 3)
        self.assertTrue(all(m.linear.x == 0 for m in self.output))
        self.move(4.0, 4.0)
        self.assertEqual(self.output[-1].linear.x, 0.5)
        self.assertEqual(self.output[-1].angular.z, 0.8)
        self.wait(lambda: self.odom[-1].pose.pose.position.x > 0)
        self.assertEqual(self.odom[-1].header.frame_id, 'odom')
        self.assertEqual(self.odom[-1].child_frame_id, 'base_link')
        self.assertEqual(self.odom[-1].pose.pose.position.z, -1.0)
        self.wait(lambda: self.output[-1].linear.x == 0, timeout=1)
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')
        count = len(self.output)
        self.pub.publish(command)
        self.wait(lambda: len(self.output) > count + 2)
        self.wait(lambda: self.output[-1].linear.x == 0)

    def test_abort_stops_and_latches(self):
        self.move()
        self.assertTrue(self.call('/safety/abort', Trigger, Trigger.Request()).success)
        self.wait(lambda: self.output[-1].linear.x == 0)
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')

    def test_invalid_command_disarms(self):
        self.move()
        msg = Twist(); msg.linear.x = math.nan
        self.pub.publish(msg)
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')
        self.wait(lambda: self.output[-1].linear.x == 0)

    def test_connection_loss_stops_and_blocks_arming(self):
        self.move()
        self.parameter('helm', 'mock_connected', False)
        self.wait(lambda: self.health[-1].status[0].message == 'DISCONNECTED')
        self.wait(lambda: self.output[-1].linear.x == 0)
        self.assertFalse(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)

    def test_telemetry_loss_stops(self):
        self.move()
        self.parameter('helm', 'mock_telemetry', False)
        self.wait(lambda: self.health[-1].status[0].message == 'DISCONNECTED')
        self.wait(lambda: self.output[-1].linear.x == 0)


class GuidanceTest(RosTest):
    def test_turn_completes_from_heading_then_hold_times_out(self):
        self.start('helm')
        self.start('wayfinder')
        status = self.observe('/navigation/status', String)
        health = self.observe('/vehicle/health', DiagnosticArray)
        odom = self.observe('/vehicle/odometry', Odometry, qos_profile_sensor_data)
        self.wait(lambda: health and odom)
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.wait_armed_delivery(health)
        handle = self.goal('TURN', timeout=9.0)
        self.assertTrue(handle.accepted)
        self.wait(lambda: any('COMPLETED' in m.data for m in status), timeout=12)
        q = odom[-1].pose.pose.orientation
        self.assertLess(abs(abs(2 * math.atan2(q.z, q.w)) - math.pi), 0.1)
        status.clear()
        handle = self.goal('HOLD', timeout=0.5)
        self.assertTrue(handle.accepted)
        self.wait(lambda: any('FAILED: timeout' in m.data for m in status), timeout=2)
        self.assertFalse(any('COMPLETED' in m.data for m in status))
