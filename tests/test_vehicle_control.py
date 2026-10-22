"""Onboard adapter contract; all vehicle endpoints are ROS fixtures or loopback."""

import math
import subprocess

from aterkia_interfaces.msg import DepthStamped, VehicleCommand
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import FluidPressure, Imu
from std_srvs.srv import SetBool, Trigger

from ros_test import RosTest


class VehicleControlTest(RosTest):
    def fields(self):
        return {v.key: v.value for v in self.health[-1].status[0].values}

    def fixture(self):
        self.armed, self.connected, self.send_sensors = False, True, True
        self.mode = 'STABILIZE'
        self.apply_mode, self.accept_mode, self.apply_arm = True, True, True
        self.arm_ack = None
        self.arm_requests = []
        self.parameters = {
            'FS_PILOT_INPUT': 2, 'FS_PILOT_TIMEOUT': 1.0,
            'SYSID_MYGCS': 255, 'FS_THR_ENABLE': 0, 'FRAME_CONFIG': 2,
        }

        async def arm(req, res):
            self.arm_requests.append(req.value)
            if req.value and self.arm_ack is not None:
                await self.arm_ack
            if self.apply_arm:
                self.armed = req.value
            res.success, res.result = True, 0
            return res

        def mode(req, res):
            res.mode_sent = self.accept_mode
            if self.apply_mode and self.accept_mode:
                self.mode = req.custom_mode
            return res

        def parameters(req, res):
            for name in req.names:
                value = self.parameters.get(name)
                if value is None:
                    res.values.append(ParameterValue())
                elif isinstance(value, int):
                    res.values.append(ParameterValue(type=2, integer_value=value))
                else:
                    res.values.append(ParameterValue(type=3, double_value=value))
            return res

        self.services = [
            self.node.create_service(CommandBool, '/mavros/cmd/arming', arm,
                                     callback_group=ReentrantCallbackGroup()),
            self.node.create_service(SetMode, '/mavros/set_mode', mode),
            self.node.create_service(GetParameters, '/mavros/param/get_parameters', parameters),
        ]
        telemetry = self.sensor_node('fcu_sensor_fixture')
        self.state_pub = telemetry.create_publisher(State, '/mavros/state', 10)
        self.imu_pub = telemetry.create_publisher(Imu, '/mavros/imu/data', qos_profile_sensor_data)
        self.pressure_pub = telemetry.create_publisher(
            FluidPressure, '/mavros/imu/static_pressure', qos_profile_sensor_data)
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.states = self.observe('/vehicle/state', State)
        self.depth = self.observe('/vehicle/depth_stamped', DepthStamped, qos_profile_sensor_data)
        self.output = self.observe('/mavros/manual_control/send', ManualControl)
        self.input = self.node.create_publisher(VehicleCommand, '/vehicle/command', 1)
        self.stamp = None
        self.source_stamps = []

        def sensors():
            self.stamp = self.node.get_clock().now().to_msg()
            self.source_stamps.append(self.stamp)
            msg = State(connected=self.connected, armed=self.armed, mode=self.mode)
            msg.header.stamp = self.stamp
            self.state_pub.publish(msg)
            if self.send_sensors:
                sample = Imu()
                sample.header.stamp = self.stamp
                sample.header.frame_id = 'base_link'
                sample.orientation.w = 1.0
                self.imu_pub.publish(sample)
                pressure = FluidPressure(fluid_pressure=119613.3)
                pressure.header = sample.header
                self.pressure_pub.publish(pressure)

        self.sensor_timer = telemetry.create_timer(0.05, sensors)
        self.processes.append(subprocess.Popen([
            'ros2', 'run', 'helm', 'helm_mavros_monitor', '--ros-args',
            '-p', 'control_enabled:=true', '-p', 'arming_enabled:=true',
            '-p', 'control_mode:=ALT_HOLD', '-p', 'firmware_version:=fixture',
            '-p', 'pressure_is_depth_sensor:=true', '-p', 'surface_pressure_pa:=100000.0',
            '-p', 'water_density_kg_m3:=1000.0', '-p', 'arming_timeout:=0.8',
            '-p', 'state_timeout:=0.6', '-p', 'telemetry_timeout:=0.5',
        ]))
        self.wait(lambda: self.health and self.states and self.depth
                  and self.input.get_subscription_count()
                  and self.fields()['preflight_ready'] == 'true'
                  and self.fields()['arming_service_ready'] == 'true')

    def prepare_arm(self):
        self.wait(lambda: not self.states[-1].armed and self.fields()['preflight_ready'] == 'true')
        response = self.call('/vehicle/set_mode', Trigger, Trigger.Request())
        self.assertTrue(response.success, response.message)
        response = self.call('/vehicle/arm', SetBool, SetBool.Request(data=True))
        self.assertTrue(response.success, response.message)

    def drive(self, values):
        self.values = values

        def send():
            msg = VehicleCommand(surge_input=self.values[0], yaw_input=self.values[1],
                                 heave_input=self.values[2])
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.input.publish(msg)

        return self.node.create_timer(0.025, send)

    @staticmethod
    def axes(msg):
        return (msg.x, msg.y, msg.z, msg.r)

    def test_depth_preserves_source_stamp_and_startup_never_arms(self):
        self.fixture()
        self.assertFalse(self.arm_requests)
        self.assertFalse(self.output)
        self.assertAlmostEqual(self.depth[-1].depth_m, 2.0)
        self.assertEqual(self.depth[-1].header.frame_id, 'base_link')
        self.assertIn(self.depth[-1].header.stamp, self.source_stamps)
        self.assertFalse(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.armed = True  # An externally armed FCU must not open a command session.
        self.wait(lambda: self.states[-1].armed)
        timer = self.drive([0.1, 0., 0.])
        mark = len(self.health)
        self.wait(lambda: len(self.health) > mark + 4)
        self.assertFalse(self.output)
        self.assertEqual(self.fields()['motion_authorized'], 'false')
        self.node.destroy_timer(timer)

    def test_axis_mapping_and_combined_input_limit(self):
        self.fixture()
        self.prepare_arm()
        timer = self.drive([0., 0., 0.])
        for values, expected in [
            ([1., 0., 0.], (200, 0, 500, 0)),
            ([-1., 0., 0.], (-200, 0, 500, 0)),
            ([0., 1., 0.], (0, 0, 500, -200)),
            ([0., -1., 0.], (0, 0, 500, 200)),
            ([0., 0., 1.], (0, 0, 600, 0)),
            ([0., 0., -1.], (0, 0, 400, 0)),
            ([1., 1., 0.], (100, 0, 500, -100)),
        ]:
            self.values = values
            self.wait(lambda: self.output and self.axes(self.output[-1]) == expected)
            self.assertEqual(self.output[-1].buttons, 0)
            self.assertEqual(self.output[-1].buttons2, 0)
            self.assertEqual(self.output[-1].enabled_extensions, 0)
        self.node.destroy_timer(timer)

    def test_first_command_and_stream_timeout_disarm_without_auto_resume(self):
        self.fixture()
        self.prepare_arm()  # No initial command must also expire.
        self.wait(lambda: False in self.arm_requests)
        self.wait(lambda: self.states[-1].armed is False)
        self.prepare_arm()
        timer = self.drive([0.1, 0., 0.])
        self.wait(lambda: self.output and self.output[-1].x > 0)
        self.node.destroy_timer(timer)
        self.wait(lambda: not self.armed)
        self.wait(lambda: self.axes(self.output[-1]) == (0, 0, 500, 0))
        mark = len(self.output)
        timer = self.drive([0.1, 0., 0.])
        ticks = len(self.health)
        self.wait(lambda: len(self.health) >= ticks + 5)
        self.assertEqual(len(self.output), mark)  # No neutral stream masking the FCU failsafe.
        self.assertEqual(self.fields()['motion_authorized'], 'false')
        self.node.destroy_timer(timer)

    def test_invalid_replayed_commands_and_multiple_publishers_disarm(self):
        self.fixture()
        for kind in ('nan', 'range', 'frame', 'old', 'future', 'negative_stamp', 'repeated', 'producer'):
            self.prepare_arm()
            msg = VehicleCommand(surge_input=0.1)
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.node.get_clock().now().to_msg()
            if kind == 'nan':
                msg.heave_input = math.nan
            elif kind == 'range':
                msg.yaw_input = 1.1
            elif kind == 'frame':
                msg.header.frame_id = 'odom'
            elif kind == 'old':
                msg.header.stamp.sec -= 2
            elif kind == 'future':
                msg.header.stamp.sec += 2
            elif kind == 'negative_stamp':
                msg.header.stamp.sec = -1
            if kind == 'producer':
                second = self.node.create_publisher(VehicleCommand, '/vehicle/command', 1)
                self.wait(lambda: self.node.count_publishers('/vehicle/command') == 2)
            self.input.publish(msg)
            if kind == 'repeated':
                self.wait(lambda: self.output and self.output[-1].x > 0)
                self.input.publish(msg)
            self.wait(lambda: self.fields()['motion_authorized'] == 'false', message=kind)
            self.wait(lambda: not self.armed)
            self.wait(lambda: not self.states[-1].armed)
            self.assertEqual(self.axes(self.output[-1]), (0, 0, 500, 0))
            if kind == 'producer':
                self.node.destroy_publisher(second)
                self.wait(lambda: self.node.count_publishers('/vehicle/command') == 1)

    def test_mode_needs_state_confirmation_and_abort_cancels_late_arm(self):
        self.fixture()
        self.apply_mode = False
        response = self.call('/vehicle/set_mode', Trigger, Trigger.Request())
        self.assertFalse(response.success)
        self.assertIn('unconfirmed', response.message)
        self.apply_mode = True
        self.assertTrue(self.call('/vehicle/set_mode', Trigger, Trigger.Request()).success)
        self.arm_ack = Future()
        client = self.node.create_client(SetBool, '/vehicle/arm')
        self.wait(client.service_is_ready)
        future = client.call_async(SetBool.Request(data=True))
        self.wait(lambda: self.arm_requests)
        self.assertTrue(self.call('/safety/abort', Trigger, Trigger.Request()).success)
        self.arm_ack.set_result(None)
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.wait(lambda: self.arm_requests == [True, False])
        self.wait(lambda: not self.states[-1].armed)
        self.assertEqual(self.fields()['motion_authorized'], 'false')
        client.destroy()

    def test_sensor_loss_mode_change_and_disconnect_latch_the_session(self):
        self.fixture()
        for fault in ('sensors', 'mode', 'connection'):
            self.prepare_arm()
            timer = self.drive([0.1, 0., 0.])
            self.wait(lambda: self.output and self.output[-1].x > 0)
            if fault == 'sensors':
                self.send_sensors = False
            elif fault == 'mode':
                self.mode = 'STABILIZE'
            else:
                self.connected = False
            self.wait(lambda: self.fields()['motion_authorized'] == 'false')
            self.wait(lambda: self.axes(self.output[-1]) == (0, 0, 500, 0))
            self.connected, self.send_sensors = True, True
            self.wait(lambda: not self.armed)
            self.wait(lambda: self.fields()['preflight_ready'] == 'true')
            self.assertEqual(self.fields()['motion_authorized'], 'false')
            self.node.destroy_timer(timer)

    def test_fcu_configuration_must_match_failsafe_sender_and_frame(self):
        self.fixture()
        for name, bad in [('FS_PILOT_INPUT', 0), ('FS_PILOT_TIMEOUT', 5.),
                          ('SYSID_MYGCS', 2), ('FS_THR_ENABLE', 1), ('FRAME_CONFIG', 1)]:
            good = self.parameters[name]
            self.parameters[name] = bad
            self.wait(lambda: self.fields()['fcu_control_config_verified'] == 'false')
            self.assertFalse(self.call('/vehicle/set_mode', Trigger, Trigger.Request()).success)
            self.assertFalse(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
            self.assertFalse(self.arm_requests)
            self.parameters[name] = good
            self.wait(lambda: self.fields()['preflight_ready'] == 'true')
