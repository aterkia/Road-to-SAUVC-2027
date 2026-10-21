import math
import subprocess
import signal
import socket
import tempfile
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from ros_test import RosTest
from test_recording import ROOT, start_recording, finish_recording


class BenchTest(RosTest):
    def test_real_mavros_udp_connection_and_no_actuator_packets(self):
        directory = Path(tempfile.mkdtemp(prefix='bench-', dir='artifacts'))
        peer_log = (directory / 'peer.log').open('w')
        launch_log = (directory / 'launch.log').open('w')
        self.addCleanup(peer_log.close)
        self.addCleanup(launch_log.close)
        peer = subprocess.Popen(['ros2', 'run', 'helm', 'helm_mavlink_peer'], stdout=peer_log, stderr=peer_log)
        self.processes.append(peer)
        self.wait(lambda: (directory / 'peer.log').read_text().strip())
        port = int((directory / 'peer.log').read_text().splitlines()[0])
        # MAVROS rejects port zero; reserve an available loopback port for launch.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            local_port = reservation.getsockname()[1]
        self.processes.append(subprocess.Popen([
            'ros2', 'launch', 'bootstrap', 'bench.launch.py',
            f'fcu_url:=udp://127.0.0.1:{local_port}@127.0.0.1:{port}'], stdout=launch_log, stderr=launch_log))
        health = self.observe('/vehicle/health', DiagnosticArray)
        heading = self.observe('/vehicle/heading', Float64, qos_profile_sensor_data)
        pressure = self.observe('/vehicle/pressure', FluidPressure, qos_profile_sensor_data)
        self.wait(lambda: heading and pressure and health and health[-1].status[0].message == 'READ_ONLY', timeout=15,
                  message=f'MAVROS connection failed; inspect {directory}')
        self.assertAlmostEqual(heading[-1].data, math.pi / 2, places=4)
        self.assertAlmostEqual(pressure[-1].fluid_pressure, 119613.3, delta=0.1)
        names = {name for name, _ in self.node.get_node_names_and_namespaces()}
        self.assertFalse({'orchestrator', 'wayfinder', 'spectra'} & names)
        # Check the actual loaded plugins, not just the YAML's intended allowlist.
        plugin_lines = (directory / 'launch.log').read_text().splitlines()
        loaded = {line.split('Plugin ')[1].split()[0] for line in plugin_lines
                  if 'Plugin ' in line and ' created' in line}
        self.assertEqual(loaded, {'sys_status', 'sys_time', 'imu'})
        recorder, recording = start_recording(self, 'bench', ROOT / 'src/bootstrap/config/bench.yaml',
            ['/mavros/state', '/mavros/imu/data', '/vehicle/state', '/vehicle/health', '/vehicle/pressure'])
        for request in (True, False):
            self.assertFalse(self.call('/vehicle/arm', SetBool, SetBool.Request(data=request)).success)
        self.assertFalse(self.call('/vehicle/set_mode', Trigger, Trigger.Request()).success)
        command = self.node.create_publisher(Twist, '/cmd_vel', 1)
        twist = Twist(); twist.linear.x = 0.3; twist.angular.z = 0.2
        timer = self.node.create_timer(0.05, lambda: command.publish(twist))
        mark = len(heading)
        self.wait(lambda: len(heading) >= mark + 10)
        self.node.destroy_timer(timer)
        received = (directory / 'peer.log').read_text().splitlines()[1:]
        self.assertTrue(received)
        # Transport may request capabilities/time, but must not emit actuator/setpoint commands.
        forbidden_ids = {11, 69, 70, 75, 82, 84, 86, 139}  # SET_MODE, manual/RC, COMMAND_INT, targets
        for line in received:
            fields = line.split()
            if not fields or fields[0] != 'RX':
                continue
            self.assertNotIn(int(fields[1]), forbidden_ids, line)
            if int(fields[1]) == 76:
                self.assertIn(int(fields[3]), {512, 520}, line)  # request message/capabilities only
        peer.send_signal(signal.SIGINT)
        peer.wait(timeout=5)
        self.processes.remove(peer)
        self.wait(lambda: health[-1].status[0].message == 'DISCONNECTED', timeout=6)
        # Leave enough observable state samples for the recorder to see the transition.
        count = len(health)
        self.wait(lambda: len(health) >= count + 3)
        manifest, bag = finish_recording(self, recorder, recording)
        self.assertEqual(manifest['configured_firmware']['firmware_family'], 'ardusub')
        self.assertEqual(manifest['configured_firmware']['firmware_version'], 'unverified')
        runtime = manifest['runtime_parameters']['/mavros_node']['stdout']
        self.assertIn(f'127.0.0.1:{port}', runtime)
        self.assertTrue(bag['/mavros/state'])
        self.assertTrue(bag['/vehicle/state'])
        self.assertTrue(bag['/vehicle/imu'])
        self.assertTrue(bag['/vehicle/pressure'])
        self.assertTrue(any(m.status[0].message == 'DISCONNECTED' for m, _ in bag['/vehicle/health']))
        self.assertIn('/mavros/rc/out', manifest['topics_without_messages'])
        print(f'Bench UDP evidence: {directory}')

    def start_monitor(self, depth=False):
        self.processes.append(subprocess.Popen([
            'ros2', 'run', 'helm', 'helm_mavros_monitor', '--ros-args',
            '-p', 'state_timeout:=0.6', '-p', 'telemetry_timeout:=0.3',
            '-p', f'pressure_is_depth_sensor:={str(depth).lower()}',
            '-p', 'surface_pressure_pa:=100000.0', '-p', 'water_density_kg_m3:=1000.0',
            '-p', 'firmware_version:=fixture', '-p', 'control_mode:=TEST_ONLY']))
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.heading = self.observe('/vehicle/heading', Float64, qos_profile_sensor_data)
        self.depth = self.observe('/vehicle/depth', Float64, qos_profile_sensor_data)
        self.imus = self.observe('/vehicle/imu', Imu, qos_profile_sensor_data)
        self.pressures = self.observe('/vehicle/pressure', FluidPressure, qos_profile_sensor_data)
        self.states = self.observe('/vehicle/state', State)
        self.state_pub = self.node.create_publisher(State, '/mavros/state', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/mavros/imu/data', qos_profile_sensor_data)
        self.pressure_pub = self.node.create_publisher(FluidPressure, '/mavros/imu/static_pressure', qos_profile_sensor_data)
        self.connected, self.send_sensors, self.send_state, self.bad, self.freeze = True, True, True, False, False
        self.saved_stamp = None
        self.sent = 0
        self.timer = self.node.create_timer(0.05, self.publish)
        self.wait(lambda: self.health and self.states and self.heading and self.pressures)

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        if self.saved_stamp is None:
            self.saved_stamp = stamp
        if self.freeze:
            stamp = self.saved_stamp
        if self.send_state:
            state = State(connected=self.connected, armed=True, manual_input=True, mode='EXTERNAL_MODE')
            state.header.stamp = stamp
            self.state_pub.publish(state)
        if self.send_sensors:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'base_link'
            imu.orientation.z = math.sin(0.6 / 2)
            imu.orientation.w = math.cos(0.6 / 2)
            if self.bad:
                imu.orientation.w = float('nan')
            self.imu_pub.publish(imu)
            pressure = FluidPressure(fluid_pressure=float('nan') if self.bad else 119613.3)
            pressure.header = imu.header
            self.pressure_pub.publish(pressure)
        self.sent += 1

    def fields(self):
        return {kv.key: kv.value for kv in self.health[-1].status[0].values}

    def test_telemetry_frames_depth_and_disabled_commands(self):
        self.start_monitor(depth=True)
        self.wait(lambda: self.depth and self.health[-1].status[0].message == 'READ_ONLY')
        self.assertAlmostEqual(self.heading[-1].data, 0.6, places=5)
        self.assertAlmostEqual(self.depth[-1].data, 2.0, places=5)
        self.assertEqual(self.imus[-1].header.frame_id, 'base_link')
        self.assertEqual(self.fields()['observed_armed'], 'true')
        self.assertEqual(self.fields()['manual_input'], 'true')
        self.assertEqual(self.fields()['configured_mode'], 'TEST_ONLY')
        self.assertNotEqual(self.health[-1].status[0].message, 'ARMED')
        for value in (True, False):
            response = self.call('/vehicle/arm', SetBool, SetBool.Request(data=value))
            self.assertFalse(response.success)
            self.assertIn('READ_ONLY', response.message)
        for service in ('/vehicle/set_mode', '/safety/abort'):
            response = self.call(service, Trigger, Trigger.Request())
            self.assertFalse(response.success)
        command = self.node.create_publisher(Twist, '/cmd_vel', 1)
        command.publish(Twist())
        # No clients/publishers to any command transport, and no fabricated odometry.
        self.wait(lambda: any(name == 'helm' for name, _ in self.node.get_node_names_and_namespaces()))
        pubs = dict(self.node.get_publisher_names_and_types_by_node('helm', '/'))
        clients = self.node.get_client_names_and_types_by_node('helm', '/')
        self.assertFalse(any(name.startswith('/mavros/') for name in pubs))
        self.assertFalse(any(name.startswith('/mavros/') for name, _ in clients))
        self.assertNotIn('/vehicle/odometry', pubs)
        self.assertNotIn('/vehicle/applied_cmd', pubs)
        self.connected = False
        self.wait(lambda: self.health[-1].status[0].message == 'DISCONNECTED')
        self.assertEqual(self.fields()['depth_available'], 'false')
        self.assertEqual(self.fields()['observed_armed'], 'unknown')

    def test_depth_is_not_inferred_from_unverified_pressure(self):
        self.start_monitor()
        self.wait(lambda: self.sent >= 10)
        self.assertFalse(self.depth)
        self.assertEqual(self.fields()['depth_available'], 'false')

    def test_invalid_sensors_and_missing_heartbeat_expire(self):
        self.start_monitor(depth=True)
        self.bad = True
        self.wait(lambda: self.health[-1].status[0].message == 'TELEMETRY_STALE')
        self.assertEqual(self.fields()['imu_fresh'], 'false')
        self.assertEqual(self.fields()['pressure_fresh'], 'false')
        imu_count, depth_count = len(self.imus), len(self.depth)
        mark = self.sent
        self.wait(lambda: self.sent >= mark + 5)
        self.assertEqual(len(self.imus), imu_count)
        self.assertEqual(len(self.depth), depth_count)
        self.send_state = False
        self.bad = False
        self.wait(lambda: self.health[-1].status[0].message == 'DISCONNECTED')

    def test_replayed_stamps_do_not_refresh_connection(self):
        self.start_monitor()
        self.freeze = True
        self.wait(lambda: self.health[-1].status[0].message == 'DISCONNECTED', timeout=2)

    def test_control_enable_and_unreferenced_depth_fail_startup(self):
        for parameters in [('control_enabled:=true',), ('pressure_is_depth_sensor:=true',)]:
            command = ['ros2', 'run', 'helm', 'helm_mavros_monitor', '--ros-args']
            for value in parameters:
                command += ['-p', value]
            result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)

    def test_launch_requires_explicit_endpoint(self):
        result = subprocess.run(['ros2', 'launch', 'bootstrap', 'bench.launch.py'],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('requires an explicit fcu_url', result.stdout + result.stderr)
