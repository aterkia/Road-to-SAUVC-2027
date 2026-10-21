#!/usr/bin/env python3
"""Bounded rosbag session with configuration evidence; never commands the vehicle."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

MIB = 1024 * 1024


def utc():
    return datetime.now(timezone.utc).isoformat()


def capture(command, timeout=4):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        return {'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'returncode': None, 'stdout': '', 'stderr': str(error)}


def snapshot(path, directory, index):
    data = path.read_bytes()
    destination = directory / f'{index:02}-{path.name}'
    destination.write_bytes(data)
    return {'source': str(path.resolve()), 'file': str(destination.relative_to(directory.parent)),
            'sha256': hashlib.sha256(data).hexdigest()}


def write_manifest(directory, manifest):
    temporary = directory / 'session.json.tmp'
    temporary.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    temporary.replace(directory / 'session.json')


def runtime_parameters(names):
    """One bounded discovery period; only read-only parameter services are called."""
    if not names:
        return {}
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters, ListParameters
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('session_parameter_snapshot')
    pending, results = {}, {}
    try:
        for name in names:
            pending[name] = {'list': node.create_client(ListParameters, name + '/list_parameters'),
                             'get': node.create_client(GetParameters, name + '/get_parameters'),
                             'future': None, 'names': None}
        deadline = time.monotonic() + 5
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for name, state in list(pending.items()):
                future = state['future']
                if future is None:
                    if state['list'].service_is_ready() and state['get'].service_is_ready():
                        state['future'] = state['list'].call_async(ListParameters.Request(depth=0))
                    continue
                if not future.done():
                    continue
                try:
                    response = future.result()
                    if state['names'] is None:
                        state['names'] = response.result.names
                        state['future'] = state['get'].call_async(GetParameters.Request(names=state['names']))
                        continue
                    if len(response.values) != len(state['names']):
                        raise ValueError('parameter response length mismatch')
                    values = {}
                    for key, value in zip(state['names'], response.values):
                        value = parameter_value_to_python(value)
                        values[key] = value.tolist() if hasattr(value, 'tolist') else value
                    results[name] = {'returncode': 0, 'stdout': yaml.safe_dump({name: {'ros__parameters': values}}),
                                     'stderr': '', 'captured_utc': utc()}
                except Exception as error:
                    results[name] = {'returncode': 1, 'stdout': '', 'stderr': str(error), 'captured_utc': utc()}
                del pending[name]
        for name in pending:
            results[name] = {'returncode': None, 'stdout': '', 'stderr': 'parameter snapshot deadline exceeded',
                             'captured_utc': utc()}
        return results
    finally:
        node.destroy_node()
        rclpy.shutdown()


def record(args):
    config = yaml.safe_load(args.recording_config.read_text())
    profile = config['profiles'][args.profile]
    topics = list(dict.fromkeys(config['common_topics'] + profile['topics'] + args.topic))
    actions = config.get('actions', [])
    nodes = profile['nodes']
    if (not topics or any(not isinstance(t, str) or not t.startswith('/') for t in topics) or
            any(not isinstance(n, str) or not n.startswith('/') for n in [*nodes, *actions])):
        raise ValueError('recording topics/nodes/actions must be absolute ROS names')
    if not math.isfinite(args.duration) or args.duration <= 0 or args.min_free_mib < 128:
        raise ValueError('duration must be positive/finite; disk reserve must be at least 128 MiB')
    # Validate evidence files before creating a session; never replace an existing session.
    for path in [args.recording_config, args.qos, *args.config]:
        if not path.is_file():
            raise ValueError(f'missing configuration: {path}')
    directory = args.output.resolve()
    directory.parent.mkdir(parents=True, exist_ok=True)
    reserve = args.min_free_mib * MIB
    if shutil.disk_usage(directory.parent).free < reserve:
        raise ValueError('insufficient free disk space for configured reserve')
    directory.mkdir()
    snapshots = directory / 'config'
    snapshots.mkdir()
    manifest = {'schema_version': 1, 'created_utc': utc(), 'profile': args.profile,
                'status': 'preparing', 'duration_limit_seconds': args.duration,
                'min_free_mib': args.min_free_mib, 'requested_topics': topics,
                'requested_actions': actions,
                'environment': {name: os.environ.get(name) for name in
                                ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTDDS_BUILTIN_TRANSPORTS')},
                'git_commit': capture(['git', 'rev-parse', 'HEAD']),
                'git_worktree': capture(['git', 'status', '--porcelain']),
                'configs': [], 'runtime_parameters': {}, 'software_versions': {}}
    write_manifest(directory, manifest)
    print(f'Preparing recording session: {directory}', flush=True)
    stop = []
    old_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, lambda number, _frame: stop.append(signal.Signals(number).name))
    recorder = None
    try:
        for path in [args.recording_config, args.qos, *args.config]:
            manifest['configs'].append(snapshot(path, snapshots, len(manifest['configs'])))
        for package in ('rosbag2_transport', 'rosbag2_storage_mcap', 'mavros', 'helm', 'spectra'):
            try:
                root = ET.parse(Path(get_package_share_directory(package)) / 'package.xml').getroot()
                manifest['software_versions'][package] = root.findtext('version')
            except (OSError, LookupError) as error:
                manifest['software_versions'][package] = f'unavailable: {error}'
        parameters = {}
        manifest['runtime_parameters'] = runtime_parameters(nodes) if not stop else {}
        for result in manifest['runtime_parameters'].values():
            if result['returncode'] == 0:
                dumped = yaml.safe_load(result['stdout'])
                if isinstance(dumped, dict):
                    for name, entry in dumped.items():
                        parameters[name.lstrip('/')] = entry.get('ros__parameters', {})
        # Preserve the actual calibration, not only its path in a parameter dump.
        calibration = parameters.get('spectra', {}).get('calibration_file')
        if calibration:
            try:
                manifest['configs'].append(snapshot(Path(calibration), snapshots, len(manifest['configs'])))
            except OSError as error:
                manifest['calibration_snapshot_error'] = str(error)
        helm = parameters.get('helm', {})
        manifest['configured_firmware'] = {key: helm.get(key, 'unavailable') for key in
                                           ('firmware_family', 'firmware_version', 'control_mode', 'control_enabled')}
        manifest['configured_firmware']['provenance'] = 'operator parameters; not verified FCU version'
        # Follow the bench's configurable raw telemetry names as well as normalized outputs.
        topics += [helm[key] for key in ('state_topic', 'imu_topic', 'pressure_topic')
                   if isinstance(helm.get(key), str) and helm[key].startswith('/') and helm[key] not in topics]
        qos = yaml.safe_load(args.qos.read_text())
        action_topics = [f'{name}/_action/{suffix}' for name in actions for suffix in
                         ('feedback', 'status', 'send_goal/_service_event',
                          'cancel_goal/_service_event', 'get_result/_service_event')]
        for topic in [*topics, *action_topics]:
            qos.setdefault(topic, {'reliability': 'best_effort', 'durability': 'volatile',
                                   'history': 'keep_last', 'depth': 20})
        qos_path = directory / 'effective-qos.yaml'
        qos_path.write_text(yaml.safe_dump(qos))
        manifest['requested_topics'] = topics
        command = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(directory / 'bag'),
                   '--storage-preset-profile', 'zstd_fast', '--max-cache-size', str(32 * MIB),
                   '--disable-keyboard-controls', '--include-hidden-topics',
                   '--qos-profile-overrides-path', str(qos_path), '--topics', *topics]
        # Lyrical filters action interfaces separately, even when explicitly named as topics.
        if actions:
            command += ['--actions', *actions]
        manifest['recorder_command'] = command
        if stop:
            manifest.update(status='interrupted_before_recording', stop_reason=stop[0])
            return 1
        if shutil.disk_usage(directory).free < reserve:
            raise ValueError('disk reserve exhausted during preparation')
        started = time.monotonic()
        manifest.update(status='recording', recording_started_utc=utc())
        write_manifest(directory, manifest)
        with (directory / 'recorder.log').open('w') as log:
            recorder = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            print(f'Recorder launched; subscription/storage logs: {directory / "recorder.log"}', flush=True)
            reason = 'recorder_exit'
            while recorder.poll() is None:
                if stop:
                    reason = stop[0]; break
                if time.monotonic() - started >= args.duration:
                    reason = 'duration_limit'; break
                if shutil.disk_usage(directory).free < reserve:
                    reason = 'disk_reserve'; break
                time.sleep(0.1)
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:
                    recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(recorder.pid, signal.SIGKILL)
                    recorder.wait()
                    reason = 'flush_timeout'
        manifest.update(stop_reason=reason, recorder_returncode=recorder.returncode,
                        recording_elapsed_seconds=time.monotonic() - started)
        # Opening with rosbag2 validates storage/plugin availability, beyond file existence.
        inspection = capture(['ros2', 'bag', 'info', str(directory / 'bag')], timeout=15)
        (directory / 'bag-info.txt').write_text(inspection['stdout'] + inspection['stderr'])
        metadata_path = directory / 'bag' / 'metadata.yaml'
        metadata = yaml.safe_load(metadata_path.read_text()) if metadata_path.exists() else {}
        info = metadata.get('rosbag2_bagfile_information', {})
        counts = {entry['topic_metadata']['name']: entry['message_count']
                  for entry in info.get('topics_with_message_count', [])}
        manifest['topic_message_counts'] = counts
        manifest['topics_without_messages'] = [topic for topic in topics if counts.get(topic, 0) == 0]
        manifest['free_bytes_at_end'] = shutil.disk_usage(directory).free
        complete = recorder.returncode == 0 and inspection['returncode'] == 0 and info.get('message_count', 0) > 0
        manifest['status'] = 'finalized' if complete else 'failed'
        print(f"Session {manifest['status']}: {directory}; stop={reason}; missing/empty={manifest['topics_without_messages']}", flush=True)
        return 0 if complete and reason in ('SIGINT', 'SIGTERM', 'duration_limit') else 1
    except Exception as error:
        manifest.update(status='failed', error=str(error))
        raise
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid, signal.SIGKILL)
                recorder.wait()
        manifest['finished_utc'] = utc()
        write_manifest(directory, manifest)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def main():
    share = Path(get_package_share_directory('bootstrap')) / 'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('mock', 'bench'), required=True)
    parser.add_argument('--output', type=Path, required=True, help='new session directory')
    parser.add_argument('--config', type=Path, action='append', default=[], help='configuration actually supplied to launch; repeatable')
    parser.add_argument('--duration', type=float, default=300.0, help='maximum recording seconds, after parameter snapshots')
    parser.add_argument('--min-free-mib', type=int, default=1024)
    parser.add_argument('--topic', action='append', default=[], help='additional/remapped sensor topic')
    parser.add_argument('--recording-config', type=Path, default=share / 'recording.yaml')
    parser.add_argument('--qos', type=Path, default=share / 'recording-qos.yaml')
    args = parser.parse_args()
    try:
        return record(args)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        parser.exit(1, f'Recording failed: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
