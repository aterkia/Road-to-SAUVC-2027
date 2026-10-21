# Aterkia

ROS 2 AUV stack; [PRD.md](PRD.md) is the requirements baseline. Implementation
and verification status are recorded in [docs/milestones.md](docs/milestones.md).

## Build and verify

Development verification uses ROS 2 Lyrical on Ubuntu 26.04. Install dependencies
on a ROS-enabled machine, then build from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
source scripts/ros_env.sh
rosdep install --from-paths src --ignore-src -r -y
bash scripts/verify.sh
source install/setup.bash
ros2 launch bootstrap auv.launch.py
```

Launch accepts `config:=/absolute/path/to/config.yaml`. Only `mode:=mock` is
implemented. Startup does not authorize motion. Hardware commissioning and a
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
mission transitions under `artifacts/qualification-*/bag` in MCAP format.
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
ros2 run spectra spectra_node --ros-args -p use_sim_time:=true
ros2 bag play artifacts/qualification-EXAMPLE/bag --clock --topics /camera/image_raw
```

Use the actual recording path printed by the qualification test. Replay only
images so recorded motion commands or mission results are not injected into a
live stack. See [runtime contracts](docs/interfaces.md) for frames, QoS, timestamp
rules and the simulation-only passage criterion.

## Current limits

M0–M3 are verified in mock simulation. M4 has tested drum/flare lab detectors
and prerequisite validation; payload and communication/localization tasks remain
unavailable. Pixhawk/ArduSub bench tests, pool data, a hardware passage criterion,
payload hardware and final rule validation are still required. See
[hardware and finals handoff](docs/hardware-and-finals.md).
