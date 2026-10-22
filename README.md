# Aterkia

ROS 2 AUV stack; [PRD.md](PRD.md) is the requirements baseline. Implementation
and verification status are recorded in [docs/milestones.md](docs/milestones.md).

## Build and verify

Development verification uses ROS 2 Lyrical on Ubuntu 26.04. Install dependencies
on a ROS-enabled machine, then build from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash &&
source scripts/ros_env.sh &&
rosdep update --rosdistro lyrical &&
sudo apt-get update &&
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y &&
sudo geographiclib-get-geoids egm96-5 &&
bash scripts/verify.sh &&
source install/setup.bash &&
ros2 launch bootstrap auv.launch.py
```

On a fresh ROS installation without `/etc/ros/rosdep/sources.list.d/20-default.list`,
run `sudo rosdep init` once before `rosdep update`. Run `rosdep update` as your
normal user; its cache is per user. The command chain stops if dependency
installation or verification fails, before sourcing or launching the workspace.
The full test suite starts a MAVROS loopback fixture, which also requires the
`egm96-5` geoid dataset even when the mission launch itself uses a mock vehicle.

An older Lyrical image can also contain message/type-support packages that are
incompatible with a newly installed MAVROS. Errors naming missing
`has_buffer_fields_*` or `cdr_*_with_endpoint_*` symbols require a consistent ROS
underlay update followed by a workspace rebuild; clearing CMake alone cannot
fix that ABI mismatch. See [MAVROS bench dependencies](docs/mavros-bench.md).

Build and symlink-install outputs contain absolute paths. They cannot be reused
between a container checkout at `/ws` and a checkout mounted at another path.
If verification reports a foreign cache, move `build/`, `install/` and `log/`
aside, then retry from a fresh terminal with only the ROS underlay sourced.

For container validation sharing the source tree, use an independent output
directory inside that container:

```bash
ATERKIA_BUILD_ROOT=/tmp/aterkia-validation bash scripts/verify.sh
source /tmp/aterkia-validation/install/setup.bash
```

Keep that output directory specific to its environment and checkout path.
Ordinary verification still uses the workspace's `build/`, `install/` and `log/`.

Launch accepts `config:=/absolute/path/to/config.yaml`. Only `mode:=mock` is
implemented for mission execution. A separate telemetry-only
[MAVROS bench launch](docs/mavros-bench.md) is available. Startup does not authorize motion. Hardware commissioning and a
physical kill switch are separate requirements; this workspace does not drive
real thrusters.

## Run the controlled qualification scenario

After building and sourcing `install/setup.bash`:

```bash
source scripts/ros_env.sh
ROS_DOMAIN_ID=77 python3 -m pytest tests/test_mission.py -k qualification -s
```

This launches the stack through bootstrap, supplies synthetic camera frames from
a fixed gate and measured mock position, authorizes the mock, then verifies
pass → U-turn → return pass. It records images, detections, telemetry, motion and
mission transitions under `artifacts/recording-mock-*/session/bag` in MCAP format,
alongside configuration and parameter evidence.
The full `scripts/verify.sh` also tests stale data, invalid commands, watchdog,
abort/cancel, competing goals, timeout and the absence of false success.
It uses ROS domain 77 by default (`ATERKIA_TEST_DOMAIN_ID` overrides this).

With a camera input available, manual mock operation is:

```bash
ros2 service call /vehicle/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
# Stop the mission, cancel its goal and disarm:
ros2 service call /mission/abort std_srvs/srv/Trigger '{}'
# Direct vehicle stop, independent of orchestrator:
ros2 service call /safety/abort std_srvs/srv/Trigger '{}'
```

The plan is `orchestrator.ros__parameters.tasks` in
`src/bootstrap/config/auv.yaml`. Runtime configuration is read-only; edit YAML
and relaunch. Failed/aborted plans never resume automatically. Re-arm and issue
a new start explicitly. `HOLD` stays at zero until cancellation or timeout; its
duration alone does not complete a mission task.

## Replay perception

In separate ROS-enabled terminals, source the workspace and `scripts/ros_env.sh`:

```bash
# Offline terminal (choose an unused domain):
ROS_DOMAIN_ID=78 ros2 run spectra spectra_node --ros-args -p use_sim_time:=true
# From a terminal configured for the original live domain:
ros2 run bootstrap replay_perception.py artifacts/recording-mock-EXAMPLE/session --domain-id 78
```

Use the actual recording path printed by the qualification test. Replay only
images so recorded motion commands or mission results are not injected into a
live stack. See [recording sessions](docs/recording.md) for mock/bench capture,
configuration snapshots, stop conditions and replay setup, and
[runtime contracts](docs/interfaces.md) for frames, QoS, timestamp
rules and the simulation-only passage criterion.

## Camera calibration and MAVROS bench

[Camera calibration](docs/camera-calibration.md) covers the offline chessboard
tool, raw versus rectified input, and paired CameraInfo for bearing guidance.
Default mock guidance remains uncalibrated; measured hardware intrinsics must
be supplied explicitly.

[MAVROS bench](docs/mavros-bench.md) covers dependencies, configurable serial/UDP
connections and validated telemetry. Arming, disarming, mode changes and motion
transmission are locked until hardware configuration is commissioned. The bench
never launches mission/guidance or fabricates horizontal odometry.

The serial endpoint defaults to `serial:///dev/ttyACM0:115200` in
`src/bootstrap/config/bench.yaml`. Adjust it once for the actual device and rebuild
`bootstrap`; launch with `ros2 launch bootstrap bench.launch.py`.

## Current limits

M0–M3 are verified in mock simulation. M4 has tested drum/flare lab detectors
and prerequisite validation; payload and communication/localization tasks remain
unavailable. Pixhawk/ArduSub bench tests, pool data, a hardware passage criterion,
payload hardware and final rule validation are still required. See
[hardware and finals handoff](docs/hardware-and-finals.md).
