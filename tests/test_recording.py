import importlib.util
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ros_test import RosTest

ROOT = Path(__file__).resolve().parents[1]


def read_bag(path, topics=None, sent_time=False):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    messages = {}
    while reader.has_next():
        topic, data, received, sent = reader.read_next_ext()
        if sent_time:
            assert sent > 0, f'DDS source timestamp unavailable for {topic}'
        messages.setdefault(topic, []).append(
            (deserialize_message(data, types[topic]), sent if sent_time else received))
    return messages


def start_recording(test, profile, config, expected, duration=120):
    Path('artifacts').mkdir(exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix=f'recording-{profile}-', dir='artifacts')) / 'session'
    process = subprocess.Popen(
        [
            'ros2',
            'run',
            'bootstrap',
            'record_session.py',
            '--profile',
            profile,
            '--output',
            str(directory),
            '--config',
            str(config),
            '--duration',
            str(duration),
        ]
    )
    test.processes.append(process)

    def subscribed():
        if not (directory / 'session.json').exists():
            return False
        if json.loads((directory / 'session.json').read_text())['status'] != 'recording':
            return False
        if ('rosbag2_recorder', '/') not in test.node.get_node_names_and_namespaces():
            return False
        subscriptions = test.node.get_subscriber_names_and_types_by_node('rosbag2_recorder', '/')
        return set(expected) <= {name for name, _ in subscriptions}

    test.wait(subscribed, timeout=40, message='recorder subscriptions not ready')
    return process, directory


def finish_recording(test, process, directory, topics=None):
    process.send_signal(signal.SIGINT)
    assert process.wait(timeout=20) == 0
    test.processes.remove(process)
    manifest = json.loads((directory / 'session.json').read_text())
    assert manifest['status'] == 'finalized'
    assert manifest['stop_reason'] == 'SIGINT'
    return manifest, read_bag(directory / 'bag', topics)


class RecordingTest(RosTest):
    def test_duration_finalizes_and_replay_only_publishes_images_on_offline_domain(self):
        # A small deterministic source exercises bag content and receive/source timestamps.
        camera = self.node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        command = self.node.create_publisher(Twist, '/cmd_vel', 1)
        status = self.node.create_publisher(String, '/navigation/status', 10)
        sent = []

        def publish():
            image = Image(height=2, width=2, encoding='bgr8', step=6, data=[0] * 12)
            image.header.frame_id = 'camera_optical_frame'
            image.header.stamp = self.node.get_clock().now().to_msg()
            sent.append(image.header.stamp.sec * 10**9 + image.header.stamp.nanosec)
            camera.publish(image)
            cmd = Twist()
            cmd.linear.x = 0.4
            command.publish(cmd)
            status.publish(String(data='fixture'))

        self.node.create_timer(0.05, publish)
        process, directory = start_recording(
            self,
            'mock',
            ROOT / 'src/bootstrap/config/auv.yaml',
            ['/camera/image_raw', '/cmd_vel'],
            duration=4,
        )
        # Continue spinning publishers while waiting for the bounded recorder to exit.
        end = time.monotonic() + 20
        while process.poll() is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(process.poll(), 0)
        self.processes.remove(process)
        manifest = json.loads((directory / 'session.json').read_text())
        self.assertEqual(manifest['stop_reason'], 'duration_limit')
        self.assertEqual(manifest['status'], 'finalized')
        self.assertIn('/vehicle/odometry', manifest['topics_without_messages'])
        self.assertNotEqual(manifest['runtime_parameters']['/helm']['returncode'], 0)
        for entry in manifest['configs']:
            self.assertEqual(
                hashlib.sha256((directory / entry['file']).read_bytes()).hexdigest(),
                entry['sha256'],
            )
        bag = read_bag(directory / 'bag')
        self.assertTrue(bag['/cmd_vel'])
        self.assertTrue(bag['/camera/image_raw'])
        for image, received in bag['/camera/image_raw']:
            source = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
            self.assertIn(source, sent)
            self.assertEqual(image.header.frame_id, 'camera_optical_frame')
            self.assertLess(abs(received - source), 500_000_000)

        # Inspect the helper on a separate domain, including absence of command publishers.
        current = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        offline_domain = 78 if current != 78 else 79
        context = Context()
        context.init(domain_id=offline_domain)
        node = rclpy.create_node('offline_replay_test', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        images = []
        node.create_subscription(Image, '/camera/image_raw', images.append, qos_profile_sensor_data)
        try:
            replay = subprocess.Popen(
                [
                    'ros2',
                    'run',
                    'bootstrap',
                    'replay_perception.py',
                    str(directory),
                    '--domain-id',
                    str(offline_domain),
                ]
            )
            self.processes.append(replay)
            end = time.monotonic() + 15
            while replay.poll() is None and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.05)
                for topic in (
                    '/cmd_vel',
                    '/vehicle/applied_cmd',
                    '/mission/status',
                    '/navigation/status',
                ):
                    self.assertEqual(node.count_publishers(topic), 0)
            self.assertEqual(replay.poll(), 0)
            self.processes.remove(replay)
            self.assertTrue(images)
            recorded = {
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                for m, _ in bag['/camera/image_raw']
            }
            self.assertTrue(
                all(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec in recorded for m in images)
            )
        finally:
            executor.shutdown()
            node.destroy_node()
            context.shutdown()


def test_invalid_limits_and_existing_output_are_rejected(tmp_path):
    base = [
        sys.executable,
        str(ROOT / 'scripts/record_session.py'),
        '--profile',
        'mock',
        '--output',
        str(tmp_path / 'session'),
    ]
    for arguments in (
        ['--duration', 'nan'],
        ['--min-free-mib', '0'],
        ['--min-free-mib', str(10**15)],
    ):
        result = subprocess.run(base + arguments, capture_output=True, text=True, timeout=10)
        assert result.returncode != 0
        assert not (tmp_path / 'session').exists()
    (tmp_path / 'session').mkdir()
    (tmp_path / 'session' / 'sentinel').write_text('keep')
    result = subprocess.run(base, capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert (tmp_path / 'session' / 'sentinel').read_text() == 'keep'


def test_replay_rejects_live_domain_and_arbitrary_topic_flags(tmp_path):
    (tmp_path / 'metadata.yaml').write_text('fixture')
    base = [sys.executable, str(ROOT / 'scripts/replay_perception.py'), str(tmp_path)]
    for arguments in (
        ['--domain-id', os.environ.get('ROS_DOMAIN_ID', '0')],
        ['--domain-id', '78', '--topics', '/cmd_vel'],
    ):
        result = subprocess.run(base + arguments, capture_output=True, text=True, timeout=5)
        assert result.returncode != 0


def test_low_disk_during_capture_requests_sigint_and_reports_failure(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'record_session', ROOT / 'scripts/record_session.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Test the runtime reserve branch without filling the user's filesystem.
    from argparse import Namespace
    from types import SimpleNamespace

    configuration = tmp_path / 'recording.yaml'
    configuration.write_text(
        'common_topics: [/cmd_vel]\nprofiles:\n  mock: {nodes: [/spectra], topics: []}\n'
    )
    calibration = tmp_path / 'camera.yaml'
    calibration.write_text('calibration fixture')
    monkeypatch.setattr(
        module,
        'runtime_parameters',
        lambda _: {
            '/spectra': {
                'returncode': 0,
                'stdout': module.yaml.safe_dump(
                    {'/spectra': {'ros__parameters': {'calibration_file': str(calibration)}}}
                ),
            }
        },
    )
    qos = tmp_path / 'qos.yaml'
    qos.write_text('{}')
    free = iter([10**12, 10**12, 0, 0])
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda _: SimpleNamespace(free=next(free)))
    monkeypatch.setattr(
        module, 'capture', lambda *a, **kw: {'returncode': 0, 'stdout': '', 'stderr': ''}
    )

    class Recorder:
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, sig):
            assert sig == signal.SIGINT
            self.returncode = 0
            bag = tmp_path / 'session/bag'
            bag.mkdir()
            (bag / 'metadata.yaml').write_text(
                'rosbag2_bagfile_information: {message_count: 1, topics_with_message_count: []}'
            )

        def wait(self, timeout):
            return self.returncode

    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **kw: Recorder())
    args = Namespace(
        profile='mock',
        output=tmp_path / 'session',
        recording_config=configuration,
        qos=qos,
        config=[],
        duration=10,
        min_free_mib=128,
        topic=[],
    )
    assert module.record(args) == 1
    manifest = json.loads((tmp_path / 'session/session.json').read_text())
    assert manifest['stop_reason'] == 'disk_reserve'
    assert manifest['recorder_returncode'] == 0
    preserved = next(entry for entry in manifest['configs'] if entry['source'] == str(calibration))
    assert (tmp_path / 'session' / preserved['file']).read_bytes() == calibration.read_bytes()
    assert preserved['sha256'] == hashlib.sha256(calibration.read_bytes()).hexdigest()
