"""Optional real ArduSub firmware tests, isolated to loopback and temporary EEPROM."""

import hashlib
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import unittest

from aterkia_interfaces.msg import DepthStamped, VehicleCommand
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import RCOut, State
from mavros_msgs.srv import MessageInterval
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import FluidPressure, Imu
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
import yaml

from ros_test import RosTest

ROOT = Path(__file__).resolve().parents[1]
BINARY = os.environ.get('ATERKIA_SITL_BINARY')
SITL_SHA256 = 'cc1ea9446f7a34388588c2398ffa17c914f43cd4d302cf187211a7bfc4204f2e'


@unittest.skipUnless(BINARY, 'Set ATERKIA_SITL_BINARY for the pinned ArduSub firmware test')
class SitlTest(RosTest):
    def fields(self):
        return {v.key: v.value for v in self.health[-1].status[0].values}

    def start_sitl(self, onboard=False):
        self.assertEqual(hashlib.sha256(Path(BINARY).read_bytes()).hexdigest(), SITL_SHA256)
        self.directory = Path(tempfile.mkdtemp(prefix='sitl-', dir=ROOT / 'artifacts'))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        config = yaml.safe_load((ROOT / 'src/bootstrap/config/bench.yaml').read_text())
        config['/mavros_node']['ros__parameters'].update(
            fcu_url=f'udp://127.0.0.1:{port}@', system_id=255, component_id=191,
            plugin_allowlist=['sys_status', 'sys_time', 'imu', 'command', 'param', 'manual_control', 'rc_io'],
            plugin_denylist=['*'],
        )
        config['/mavros']['ros__parameters'].update(
            system_id=255, component_id=191,
            plugin_allowlist=['sys_status', 'sys_time', 'imu', 'command', 'param', 'manual_control', 'rc_io'],
        )
        config['helm']['ros__parameters'].update(
            control_enabled=True, arming_enabled=True, firmware_version='4.7.2-SITL-fb773567',
            control_mode='ALT_HOLD', pressure_is_depth_sensor=True,
            surface_pressure_pa=101325., water_density_kg_m3=1000.,
        )
        if onboard:
            config['wayfinder'] = {'ros__parameters': {
                'target_depth_m': 0.8, 'settle_time_s': 0.4,
            }}
            config['orchestrator'] = {'ros__parameters': {
                'tasks': ['DIVE_TO_DEPTH', 'TURN'], 'start_delay_s': 20.0,
                'turn_angle': 0.4, 'task_timeout': 45.0,
            }}
        config_file = self.directory / 'config.yaml'
        self.config_file = config_file
        config_file.write_text(yaml.safe_dump(config))
        processes = [
            ('firmware', [str(Path(BINARY).resolve()), '--model', 'vectored_6dof',
                          '--home', '-35.363262,149.165237,0,0',
                          '--defaults', str(ROOT / 'tests/sitl/ardusub.parm'),
                          '--serial0', f'udpclient:127.0.0.1:{port}',
                          '--serial1', 'none', '--serial2', 'none']),
            ('mavros', ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '--params-file', str(config_file)]),
            ('helm', ['ros2', 'run', 'helm', 'helm_mavros_monitor', '--ros-args', '--params-file', str(config_file)]),
        ]
        if onboard:
            processes = processes[:1] + [('launch', [
                'ros2', 'launch', 'bootstrap', 'auv.launch.py', 'mode:=hardware',
                f'config:={config_file}',
            ])]
        for name, args in processes:
            log = (self.directory / f'{name}.log').open('w')
            self.addCleanup(log.close)
            process = subprocess.Popen(args, cwd=self.directory, stdout=log, stderr=log)
            self.processes.append(process)
            if name == 'helm':
                self.helm = process
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.states = self.observe('/vehicle/state', State)
        self.depth = self.observe('/vehicle/depth_stamped', DepthStamped, qos_profile_sensor_data)
        self.pressure = self.observe('/vehicle/pressure', FluidPressure, qos_profile_sensor_data)
        self.imus = self.observe('/vehicle/imu', Imu, qos_profile_sensor_data)
        raw_imu = self.observe('/mavros/imu/data', Imu, qos_profile_sensor_data)
        raw_pressure = self.observe('/mavros/imu/static_pressure', FluidPressure, qos_profile_sensor_data)
        motor_output = self.observe('/mavros/rc/out', RCOut, qos_profile_sensor_data)
        self.fcu_outputs = motor_output
        self.addCleanup(lambda: (self.directory / 'telemetry.txt').write_text(
            f'Health: {self.health[-1:]!s}\nRaw IMU: {raw_imu[-1:]!s}\n'
            f'Raw pressure: {raw_pressure[-1:]!s}\nDepth: {self.depth[-1:]!s}\n'
            f'FCU outputs: {motor_output[-1:]!s}\n'))
        self.wait(lambda: self.states and self.states[-1].connected, timeout=15)
        for message_id, rate in ((29, 10.0), (30, 20.0)):
            response = self.call('/mavros/set_message_interval', MessageInterval,
                MessageInterval.Request(message_id=message_id, message_rate=rate))
            self.assertTrue(response.success, f'SITL telemetry interval rejected: {message_id}')
        self.wait(lambda: self.health and self.states and self.depth and self.imus
                  and self.fields()['preflight_ready'] == 'true', timeout=45,
                  message=f'SITL preflight not ready; inspect {self.directory}')
        print(f'ArduSub SITL evidence: {self.directory}')

    def test_alt_hold_depth_yaw_feedback_and_firmware_input_failsafe(self):
        self.start_sitl()
        self.input = self.node.create_publisher(VehicleCommand, '/vehicle/command', 1)
        self.assertFalse(self.states[-1].armed)
        self.assertTrue(self.call('/vehicle/set_mode', Trigger, Trigger.Request()).success)
        response = self.call('/vehicle/arm', SetBool, SetBool.Request(data=True))
        self.assertTrue(response.success, response.message)
        self.values = [0., 0., -0.2]

        def command():
            msg = VehicleCommand(surge_input=self.values[0], yaw_input=self.values[1],
                                 heave_input=self.values[2])
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.input.publish(msg)

        timer = self.node.create_timer(0.05, command)
        initial_pressure = self.pressure[-1].fluid_pressure
        initial_depth = self.depth[-1].depth_m
        self.wait(lambda: self.depth[-1].depth_m > initial_depth + 0.3, timeout=15,
                  message=f'Down input did not increase measured water depth: {self.directory}')
        self.assertGreater(self.pressure[-1].fluid_pressure, initial_pressure + 2000)
        self.values = [0., 0.2, 0.]
        q = self.imus[-1].orientation
        import math
        heading = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        initial_heading = heading(q)
        def yaw_change():
            delta = heading(self.imus[-1].orientation) - initial_heading
            return math.atan2(math.sin(delta), math.cos(delta))
        self.wait(lambda: yaw_change() > 0.15, timeout=15,
                  message=f'Positive FLU yaw did not turn counterclockwise: {self.directory}')
        self.values = [0., 0., 0.]
        # Kill helm alone: MAVROS keeps sending heartbeats, so only pilot-input failsafe can stop the FCU.
        self.helm.send_signal(signal.SIGINT)
        self.helm.wait(timeout=5)
        self.processes.remove(self.helm)
        self.node.destroy_timer(timer)
        raw_states = self.observe('/mavros/state', State)
        self.wait(lambda: raw_states and not raw_states[-1].armed, timeout=5,
                  message=f'Firmware did not disarm after guidance transport died: {self.directory}')

    def start_onboard_mission(self, full_launch=False):
        self.start_sitl(onboard=full_launch)
        self.commands = self.observe('/vehicle/command', VehicleCommand)
        self.mission = self.observe('/mission/status', String,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.navigation = self.observe('/navigation/status', String)
        self.assertFalse(self.states[-1].armed)
        if not full_launch:
            self.start('wayfinder', ('vehicle_mode', 'hardware'), ('target_depth_m', 0.8),
                       ('settle_time_s', 0.4))
            self.wayfinder = self.processes[-1]
            self.start('orchestrator', ('vehicle_mode', 'hardware'),
                       ('tasks', '[DIVE_TO_DEPTH, TURN]'), ('start_delay_s', 2.0),
                       ('turn_angle', 0.4), ('task_timeout', 45.0))
        self.wait(lambda: self.mission and self.mission[-1].data.startswith('COUNTDOWN'))
        self.assertFalse(self.call('/mission/start', Trigger, Trigger.Request()).success)

    def test_onboard_countdown_depth_turn_and_confirmed_disarm(self):
        self.start_onboard_mission(full_launch=True)
        self.wait(lambda: any(m.data == 'COMPLETED: disarm confirmed' for m in self.mission),
                  timeout=95, message=f'Onboard mission did not complete: {self.directory}')
        completed = [m.data for m in self.mission if m.data.startswith('TASK_COMPLETED')]
        self.assertEqual(completed, ['TASK_COMPLETED: 1 DIVE_TO_DEPTH', 'TASK_COMPLETED: 2 TURN'])
        self.assertTrue(any(c.heave_input < 0 for c in self.commands))
        self.assertTrue(any(c.yaw_input > 0 for c in self.commands))
        self.assertTrue(all(c.surge_input == 0 for c in self.commands))
        self.wait(lambda: not self.states[-1].armed)
        self.assertFalse(self.states[-1].armed)
        self.assertEqual(self.node.count_publishers('/vehicle/command'), 1)
        (self.directory / 'mission.txt').write_text('\n'.join(m.data for m in self.mission))

    def test_wayfinder_loss_disarms_while_mavros_keeps_heartbeat(self):
        self.start_onboard_mission()
        self.wait(lambda: self.states[-1].armed and any(c.heave_input < 0 for c in self.commands),
                  timeout=10)
        self.wayfinder.send_signal(signal.SIGINT)
        self.wayfinder.wait(timeout=5)
        self.processes.remove(self.wayfinder)
        self.wait(lambda: not self.states[-1].armed, timeout=5)
        self.wait(lambda: any(m.data == 'FAILED: disarm confirmed' for m in self.mission))
        self.assertEqual(self.fields()['motion_authorized'], 'false')
        self.assertTrue(self.states[-1].connected)

    def test_camera_to_onboard_approach_with_actual_fcu_feedback_and_bag(self):
        import math
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from test_calibration import calibration_data
        from test_vision import gate_image
        from test_recording import start_recording, finish_recording

        self.start_sitl()
        calibration = self.directory / 'camera.yaml'
        calibration.write_text(yaml.safe_dump(calibration_data()))
        self.commands = self.observe('/vehicle/command', VehicleCommand)
        self.mission = self.observe('/mission/status', String,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        navigation = self.observe('/navigation/status', String)
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        bridge = CvBridge()
        frame = [np.zeros((480, 640, 3), dtype=np.uint8)]

        def image():
            message = bridge.cv2_to_imgmsg(frame[0], encoding='bgr8')
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = 'camera_optical_frame'
            camera.publish(message)

        self.node.create_timer(0.1, image)
        self.start('spectra', ('calibration_file', str(calibration)), ('input_geometry', 'rectified'))
        self.start('wayfinder', ('vehicle_mode', 'hardware'), ('target_depth_m', 0.8),
                   ('settle_time_s', 0.3), ('use_camera_info', 'true'), ('require_camera_on_start', 'true'))
        self.wait_navigation_ready()
        self.start('orchestrator', ('vehicle_mode', 'hardware'), ('tasks', '[DIVE_TO_DEPTH, APPROACH_GATE]'),
                   ('start_delay_s', 15.0), ('task_timeout', 45.0))
        recorder, session = start_recording(self, 'autonomous', self.config_file,
            ['/vehicle/command', '/vehicle/depth_stamped', '/perception/detections',
             '/camera/image_raw', '/mission/status', '/mavros/rc/out'])
        self.wait(lambda: any('RUNNING: SEARCH' == s.data for s in navigation)
                  and self.commands[-1].yaw_input > 0, timeout=30)

        def heading():
            q = self.imus[-1].orientation
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        initial = heading()
        self.wait(lambda: math.atan2(math.sin(heading()-initial), math.cos(heading()-initial)) > 0.08,
                  timeout=10)
        frame[0] = gate_image(center=450, width=200)
        self.wait(lambda: self.commands[-1].yaw_input < 0)
        self.wait(lambda: self.imus[-1].angular_velocity.z < -0.02, timeout=10)
        frame[0] = gate_image(center=400, width=200)
        self.wait(lambda: self.commands[-1].surge_input > 0)
        # Forward mixer component for the pinned Vectored-6DoF SITL frame.
        # This confirms FCU demand, not physical displacement or gate crossing.
        def forward_output():
            if not self.fcu_outputs or len(self.fcu_outputs[-1].channels) < 4:
                return False
            a, b, c, d = self.fcu_outputs[-1].channels[:4]
            return c + d - a - b > 40
        self.wait(forward_output)
        frame[0] = gate_image(center=400, width=400)
        self.wait(lambda: any(s.data == 'COMPLETED: disarm confirmed' for s in self.mission), timeout=10)
        self.wait(lambda: not self.states[-1].armed)
        manifest, bag = finish_recording(self, recorder, session,
            ['/vehicle/command', '/vehicle/depth_stamped', '/mission/status', '/mavros/rc/out'])
        self.assertTrue(any(s.data == 'TASK_COMPLETED: 2 APPROACH_GATE' for s in self.mission))
        self.assertTrue(any(c.surge_input > 0 for c, _ in bag['/vehicle/command']))
        self.assertTrue(bag['/vehicle/depth_stamped'])
        self.assertTrue(bag['/mavros/rc/out'])
        self.assertEqual(manifest['profile'], 'autonomous')
        self.assertFalse(any('PASS_GATE' in s.data for s in self.mission))
        (self.directory / 'mission.txt').write_text('\n'.join(s.data for s in self.mission))
        print(f'Visual autonomous recording: {session}')
