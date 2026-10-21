"""Verify build isolation and fail-fast behavior without starting ROS nodes."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def verification_workspace(tmp_path):
    workspace = tmp_path / 'workspace with spaces'
    scripts = workspace / 'scripts'
    scripts.mkdir(parents=True)
    for name in ('verify.sh', 'ros_env.sh'):
        shutil.copyfile(ROOT / 'scripts' / name, scripts / name)

    binaries = tmp_path / 'bin'
    binaries.mkdir()
    colcon = binaries / 'colcon'
    colcon.write_text('''#!/usr/bin/env python3
import json, os, pathlib, sys
with open(os.environ['VERIFY_CALLS'], 'a') as stream:
    stream.write(json.dumps({'args': sys.argv[1:], 'cwd': os.getcwd(),
                             'domain': os.environ['ROS_DOMAIN_ID'],
                             'overlay': os.environ.get('VERIFY_OVERLAY')}) + '\\n')
if 'build' in sys.argv:
    if os.environ.get('VERIFY_FAIL_BUILD'):
        raise SystemExit(3)
    target = pathlib.Path(sys.argv[sys.argv.index('--install-base') + 1])
    target.mkdir(parents=True)
    (target / 'setup.bash').write_text('export VERIFY_OVERLAY=loaded\\n')
''')
    colcon.chmod(0o755)
    environment = dict(
        os.environ,
        PATH=str(binaries) + os.pathsep + os.environ['PATH'],
        VERIFY_CALLS=str(tmp_path / 'calls.jsonl'),
        ATERKIA_TEST_DOMAIN_ID='83',
    )
    environment.pop('ATERKIA_BUILD_ROOT', None)
    environment.pop('VERIFY_OVERLAY', None)
    return workspace, environment


@pytest.mark.parametrize('custom_output', [False, True])
def test_verify_uses_one_output_root_for_build_source_and_tests(
    verification_workspace, tmp_path, custom_output
):
    workspace, environment = verification_workspace
    output = tmp_path / 'isolated output' if custom_output else workspace
    if custom_output:
        environment['ATERKIA_BUILD_ROOT'] = str(output)
    cache = output / 'build/helm/CMakeCache.txt'
    cache.parent.mkdir(parents=True)
    cache.write_text(
        f'CMAKE_CACHEFILE_DIR:INTERNAL={cache.parent}\n'
        f'CMAKE_HOME_DIRECTORY:INTERNAL={workspace}/src/helm\n'
    )
    result = subprocess.run(
        ['bash', str(workspace / 'scripts/verify.sh')],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    calls = [
        json.loads(line) for line in Path(environment['VERIFY_CALLS']).read_text().splitlines()
    ]
    assert len(calls) == 3
    assert all(call['cwd'] == str(workspace) and call['domain'] == '83' for call in calls)
    assert calls[0]['args'] == [
        '--log-base',
        str(output / 'log'),
        'build',
        '--build-base',
        str(output / 'build'),
        '--install-base',
        str(output / 'install'),
        '--symlink-install',
        '--parallel-workers',
        '2',
    ]
    assert calls[1]['overlay'] == 'loaded'
    assert calls[1]['args'] == [
        '--log-base',
        str(output / 'log'),
        'test',
        '--build-base',
        str(output / 'build'),
        '--install-base',
        str(output / 'install'),
        '--event-handlers',
        'console_direct+',
    ]
    assert calls[2]['args'] == [
        'test-result',
        '--test-result-base',
        str(output / 'build'),
        '--verbose',
    ]
    if custom_output:
        assert not (workspace / 'build').exists()
        assert not (workspace / 'install').exists()


def test_foreign_cache_stops_before_build_and_preserves_outputs(verification_workspace):
    workspace, environment = verification_workspace
    cache = workspace / 'build/helm/CMakeCache.txt'
    cache.parent.mkdir(parents=True)
    original = (
        'CMAKE_CACHEFILE_DIR:INTERNAL=/ws/build/helm\nCMAKE_HOME_DIRECTORY:INTERNAL=/ws/src/helm\n'
    )
    cache.write_text(original)
    result = subprocess.run(
        ['bash', str(workspace / 'scripts/verify.sh')],
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert 'another workspace path' in result.stderr
    assert 'ATERKIA_BUILD_ROOT' in result.stderr
    assert cache.read_text() == original
    assert not Path(environment['VERIFY_CALLS']).exists()


def test_failed_build_does_not_source_or_run_tests(verification_workspace):
    workspace, environment = verification_workspace
    environment['VERIFY_FAIL_BUILD'] = '1'
    result = subprocess.run(
        ['bash', str(workspace / 'scripts/verify.sh')],
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 3
    calls = Path(environment['VERIFY_CALLS']).read_text().splitlines()
    assert len(calls) == 1
    assert not (workspace / 'install').exists()
