import math
from pathlib import Path
import subprocess
import tempfile
import time
import yaml
from ament_index_python.packages import get_package_share_directory

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from vision_msgs.msg import Detection2DArray

from mock_scene import GateScene
from ros_test import RosTest
from test_recording import start_recording, finish_recording, read_bag


class MissionTest(RosTest):
    def stack(self, tasks=None, timeout=20.0):
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.output = self.observe('/vehicle/applied_cmd', Twist)
        self.odom = self.observe('/vehicle/odometry', Odometry, qos_profile_sensor_data)
        self.status = self.observe(
            '/mission/status',
            String,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.navigation = self.observe('/navigation/status', String)
        self.detections = self.observe(
            '/perception/detections', Detection2DArray, qos_profile_sensor_data
        )
        self.scene = GateScene(self.node)
        config = yaml.safe_load(
            (Path(get_package_share_directory('bootstrap')) / 'config' / 'auv.yaml').read_text()
        )
        config['orchestrator']['ros__parameters']['task_timeout'] = timeout
        if tasks is not None:
            config['orchestrator']['ros__parameters']['tasks'] = tasks
        directory = tempfile.TemporaryDirectory(prefix='aterkia-test-')
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'auv.yaml'
        path.write_text(yaml.safe_dump(config))
        self.config_path = path
        self.processes.append(
            subprocess.Popen(['ros2', 'launch', 'bootstrap', 'auv.launch.py', f'config:={path}'])
        )
        self.wait(lambda: self.health and self.odom and self.detections and self.status, timeout=20)

    def authorize(self):
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.wait_armed_delivery(self.health)

    def start_mission(self):
        self.authorize()
        self.wait_navigation_ready()
        self.assertTrue(self.call('/mission/start', Trigger, Trigger.Request()).success)

    def test_qualification_pass_turn_return_with_evidence_and_bag(self):
        self.stack()
        recorder, session = start_recording(
            self,
            'mock',
            self.config_path,
            [
                '/camera/image_raw',
                '/perception/detections',
                '/cmd_vel',
                '/vehicle/applied_cmd',
                '/vehicle/health',
                '/mission/status',
                '/navigation/task/_action/feedback',
                '/navigation/task/_action/status',
            ],
        )
        self.start_mission()
        self.wait(lambda: any(m.data == 'COMPLETED' for m in self.status), timeout=40)
        completed = [m.data for m in self.status if m.data.startswith('TASK_COMPLETED')]
        self.assertEqual(
            completed,
            [
                'TASK_COMPLETED: 1 PASS_GATE',
                'TASK_COMPLETED: 2 TURN',
                'TASK_COMPLETED: 3 PASS_GATE',
            ],
        )
        self.assertGreater(max(m.pose.pose.position.x for m in self.odom), 1.5)
        self.assertLess(self.odom[-1].pose.pose.position.x, 0.6)
        q = self.odom[-1].pose.pose.orientation
        self.assertLess(abs(abs(2 * math.atan2(q.z, q.w)) - math.pi), 0.1)
        self.assertTrue(any('PASS' in m.data for m in self.navigation))
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')
        self.wait(lambda: self.output[-1].linear.x == 0)
        manifest, bag = finish_recording(
            self,
            recorder,
            session,
            [
                '/cmd_vel',
                '/vehicle/applied_cmd',
                '/vehicle/health',
                '/mission/status',
                '/vehicle/odometry',
            ],
        )
        for topic in (
            '/camera/image_raw',
            '/perception/detections',
            '/cmd_vel',
            '/vehicle/applied_cmd',
            '/vehicle/health',
            '/mission/status',
            '/navigation/task/_action/feedback',
        ):
            self.assertGreater(manifest['topic_message_counts'][topic], 0)
        self.assertTrue(
            all(result['returncode'] == 0 for result in manifest['runtime_parameters'].values())
        )
        self.assertTrue(any(m.data == 'COMPLETED' for m, _ in bag['/mission/status']))
        self.assertTrue(any(m.status[0].message == 'ARMED' for m, _ in bag['/vehicle/health']))
        self.assertTrue(any(m.linear.x > 0 for m, _ in bag['/cmd_vel']))
        self.assertTrue(any(m.linear.x > 0 for m, _ in bag['/vehicle/applied_cmd']))
        # Recorder discovery can delay receipt; watchdog production timing uses DDS source stamps.
        sent_commands = read_bag(session / 'bag', ['/cmd_vel'], sent_time=True)
        times = [t for _, t in sent_commands['/cmd_vel']]
        max_gap = max(b - a for a, b in zip(times, times[1:])) / 1e9
        self.assertLess(max_gap, 0.4, 'command source gap exceeded mock watchdog budget')
        print(
            f'Qualification recording: {session}; max command source gap={max_gap:.3f}s',
            flush=True,
        )

    def test_abort_during_motion_does_not_auto_resume(self):
        self.stack()
        self.start_mission()
        self.wait(lambda: self.output[-1].linear.x > 0)
        self.assertTrue(self.call('/mission/abort', Trigger, Trigger.Request()).success)
        self.wait(lambda: any(m.data.startswith('ABORTED') for m in self.status))
        self.wait(lambda: self.output[-1].linear.x == 0)
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')
        self.assertFalse(self.call('/mission/start', Trigger, Trigger.Request()).success)
        terminal_count = len(self.status)
        end = time.monotonic() + 0.6
        self.wait(lambda: time.monotonic() >= end, timeout=1)
        self.assertFalse(any(m.data.startswith('RUNNING') for m in self.status[terminal_count:]))
        self.assertFalse(any(m.data.startswith('TASK_COMPLETED') for m in self.status))

    def test_timeout_never_advances_to_next_task(self):
        self.stack(tasks=['HOLD', 'TURN'], timeout=0.5)
        self.start_mission()
        self.wait(lambda: any(m.data.startswith('FAILED') for m in self.status), timeout=3)
        self.assertFalse(any('2/2' in m.data or m.data == 'COMPLETED' for m in self.status))
        self.wait(lambda: self.output[-1].linear.x == 0)

    def test_camera_loss_aborts_plan_and_stops(self):
        self.stack()
        self.start_mission()
        self.wait(lambda: self.output[-1].linear.x > 0)
        self.scene.enabled = False
        self.wait(lambda: any(m.data.startswith('FAILED') for m in self.status), timeout=3)
        self.wait(lambda: self.output[-1].linear.x == 0)
        self.assertFalse(any('TASK_COMPLETED' in m.data for m in self.status))

    def test_action_rejects_second_goal_and_supports_cancel(self):
        self.stack()
        self.authorize()
        first = self.goal('HOLD')
        self.assertTrue(first.accepted)
        self.assertFalse(self.goal('TURN').accepted)
        future = first.cancel_goal_async()
        self.wait(future.done)
        result = first.get_result_async()
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result().result.status, 'ABORTED')
        self.assertFalse(self.goal('DROP').accepted)
        self.assertFalse(self.goal('TURN', timeout=float('nan')).accepted)
