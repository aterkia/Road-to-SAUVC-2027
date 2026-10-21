import math
from pathlib import Path
import signal
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


class MissionTest(RosTest):
    def stack(self, tasks=None, timeout=20.0):
        self.health = self.observe('/vehicle/health', DiagnosticArray)
        self.output = self.observe('/vehicle/applied_cmd', Twist)
        self.odom = self.observe('/vehicle/odometry', Odometry, qos_profile_sensor_data)
        self.status = self.observe('/mission/status', String,
                                  QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.navigation = self.observe('/navigation/status', String)
        self.detections = self.observe('/perception/detections', Detection2DArray, qos_profile_sensor_data)
        self.scene = GateScene(self.node)
        config = yaml.safe_load((Path(get_package_share_directory('bootstrap')) / 'config' / 'auv.yaml').read_text())
        config['orchestrator']['ros__parameters']['task_timeout'] = timeout
        if tasks is not None:
            config['orchestrator']['ros__parameters']['tasks'] = tasks
        directory = tempfile.TemporaryDirectory(prefix='aterkia-test-')
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / 'auv.yaml'
        path.write_text(yaml.safe_dump(config))
        self.processes.append(subprocess.Popen(
            ['ros2', 'launch', 'bootstrap', 'auv.launch.py', f'config:={path}']))
        self.wait(lambda: self.health and self.odom and self.detections and self.status)

    def authorize(self):
        self.assertTrue(self.call('/vehicle/arm', SetBool, SetBool.Request(data=True)).success)
        self.wait(lambda: self.health[-1].status[0].message == 'ARMED')

    def start_mission(self):
        self.authorize()
        self.assertTrue(self.call('/mission/start', Trigger, Trigger.Request()).success)

    def test_qualification_pass_turn_return_with_evidence_and_bag(self):
        self.stack()
        # Record a real ROS session as inspectable evidence. No real actuators exist.
        Path('artifacts').mkdir(exist_ok=True)
        recording = Path(tempfile.mkdtemp(prefix='qualification-', dir='artifacts')) / 'bag'
        recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(recording),
                                     '--storage-preset-profile', 'zstd_fast', '--topics',
                                     '/camera/image_raw', '/perception/detections',
                                     '/vehicle/odometry', '/vehicle/applied_cmd',
                                     '/mission/status', '/navigation/status'])
        self.processes.append(recorder)
        self.wait(lambda: any('rosbag2_recorder' in n for n in self.node.get_node_names()))
        self.start_mission()
        self.wait(lambda: any(m.data == 'COMPLETED' for m in self.status), timeout=40)
        completed = [m.data for m in self.status if m.data.startswith('TASK_COMPLETED')]
        self.assertEqual(completed, ['TASK_COMPLETED: 1 PASS_GATE', 'TASK_COMPLETED: 2 TURN',
                                     'TASK_COMPLETED: 3 PASS_GATE'])
        self.assertGreater(max(m.pose.pose.position.x for m in self.odom), 1.5)
        self.assertLess(self.odom[-1].pose.pose.position.x, 0.6)
        q = self.odom[-1].pose.pose.orientation
        self.assertLess(abs(abs(2*math.atan2(q.z, q.w))-math.pi), 0.1)
        self.assertTrue(any('PASS' in m.data for m in self.navigation))
        self.wait(lambda: self.health[-1].status[0].message == 'DISARMED')
        self.wait(lambda: self.output[-1].linear.x == 0)
        recorder.send_signal(signal.SIGINT)
        self.assertEqual(recorder.wait(timeout=10), 0)
        self.processes.remove(recorder)
        self.assertTrue((recording / 'metadata.yaml').exists())
        print(f'Qualification recording: {recording}', flush=True)

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
