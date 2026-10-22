# Recording and perception replay

This implements research idea 3 within `scripts/` and `bootstrap`. PRD boundaries
and vehicle authorization remain unchanged. There is no new runtime package.

## Capture a session

Build/source the workspace, source `scripts/ros_env.sh`, and use the same
`ROS_DOMAIN_ID` as the stack. Start the stack before preparing the recorder;
start the experiment only after `recorder.log` shows the required subscriptions.
For a mock stack launched with `src/bootstrap/config/auv.yaml`:

```bash
ros2 run bootstrap record_session.py --profile mock \
  --config src/bootstrap/config/auv.yaml \
  --output artifacts/mock-session-001 --duration 300 --min-free-mib 1024
```

For the separate [MAVROS bench](mavros-bench.md):

```bash
ros2 run bootstrap record_session.py --profile bench \
  --config src/bootstrap/config/bench.yaml \
  --output artifacts/bench-session-001 --duration 300 --min-free-mib 1024
```

Use a new output directory each time; existing sessions are never overwritten.
Pass the files actually supplied to launch, repeating `--config` for overrides.
The tool also reads live ROS parameters, so launch argument overrides such as
`fcu_url` are preserved independently of the YAML copy. A missing node/service
is reported as an unavailable snapshot after a shared five-second deadline;
it does not prevent recording the sources that are available.

`src/bootstrap/config/recording.yaml` defines three explicit capture profiles:

| Capture | Sources |
| --- | --- |
| Common | Camera/CameraInfo, detections/paired CameraInfo, pre-adapter `/cmd_vel` if present, health, depth, heading, mission/navigation status, navigation Action feedback/status, parameter events and ROS logs |
| Mock | Ground-truth odometry and `/vehicle/applied_cmd` |
| Bench | Normalized and raw MAVROS state, IMU, pressure, plus battery, status text, diagnostics and RC feedback if published |
| Autonomous | Stamped vehicle command/depth, state, IMU, pressure, raw MAVROS telemetry, status text, diagnostics and optional FCU RC output; parameters of mission/guidance/adapter and MAVROS parameter cache |

Use `--profile autonomous --config src/bootstrap/config/autonomous.yaml` for the
[onboard mission](autonomous.md), with the usual output/duration options. FCU RC
output is an autopilot demand, not measured thruster motion or passage evidence.

Bench raw state/IMU/pressure topic overrides are added from live helm parameters.
Use `--topic /additional_sensor` for another source, or `--recording-config` for
a custom capture list. Action interfaces require the separate `actions` list on
Lyrical. Service event topics can also be captured if Action introspection has
already been enabled elsewhere; this tool does not enable introspection or
promise a complete goal/request/result trace. Mission status and Action
feedback/status provide the current execution evidence.

The recorder never enables MAVROS plugins or calls arm, mode, motion or mission
services. RC output is normally absent from the locked bench profile. Missing
RC feedback must not be interpreted as zero thruster output. No measured
hardware actuator response is currently available.

## Evidence and stopping

Each session contains:

- `bag/`: MCAP with `zstd_fast` chunk compression, reopenable with `ros2 bag info`.
- `session.json`: requested topics/actions, observed message counts, missing or
  empty topics, timestamps, stop reason, software package versions, ROS environment,
  Git commit/dirty-file status and live parameter snapshots.
- `config/`: supplied YAML files and the live spectra calibration file when
  configured, with SHA-256 hashes in the manifest. Snapshot failures are explicit.
- `effective-qos.yaml`, `recorder.log` and `bag-info.txt`: the actual recorder
  settings, subscription/storage diagnostics and final storage inspection.

`configured_firmware` is operator-supplied metadata, **not an independently read
FCU version**. Unknown versions remain unverified. Git status identifies a dirty
worktree but does not archive its source diff; preserve the matching source
revision separately before sharing an experiment for reproduction. Supplied
configuration copies may contain connection details, so review them before sharing.

Ctrl-C/SIGINT, SIGTERM and the duration limit request SIGINT on rosbag2 and wait
up to 15 seconds for flushing. Duration starts after snapshots and includes
recorder startup/discovery. It limits the session; rosbag2's `--max-bag-duration`
would instead split files. A hung recorder is killed and reported as failed.

Free space is checked before recording and every 0.1 seconds. Crossing the
configured reserve stops and flushes the recorder, returns a nonzero exit code,
and records `disk_reserve`. The minimum reserve is 128 MiB, default 1024 MiB;
the cache setting is 32 MiB (rosbag2 may use two buffers). This is a stop threshold,
not a filesystem quota or a guarantee against other processes exhausting disk.
An empty/unreadable bag or recorder error also returns a nonzero exit code.

`status: finalized` means a nonempty bag reopened successfully; inspect the stop
reason and `topics_without_messages` before accepting an experiment. It does not
certify that every requested stream was present or lossless. Best-effort capture
subscriptions avoid reliable-delivery backpressure; bounded queues can drop data.
Compression, storage and discovery still consume CPU/I/O. The mock qualification
test checks recorded command gaps against its 0.4-second watchdog budget; repeat
performance measurements with the real camera rate/resolution and onboard disk.

Stamped messages retain their source header. Bag receive timestamps allow ordering
unstamped `Twist` and status messages, but include transport/scheduling delay and
are not source acquisition timestamps. Correlating multiple machines additionally
requires clock synchronization. Mock timestamp checks do not establish hardware
clock accuracy.

## Replay only camera input

Choose an unused offline domain, distinct from the live stack. In an offline
terminal (after sourcing the workspace and ROS environment), run spectra only:

```bash
ROS_DOMAIN_ID=78 ros2 run spectra spectra_node --ros-args -p use_sim_time:=true
```

From a terminal still configured for the original live domain, run:

```bash
ros2 run bootstrap replay_perception.py artifacts/mock-session-001 --domain-id 78
```

The helper accepts a session or bag directory, sets the chosen domain for the
player, and publishes only `/camera/image_raw` plus `/clock`. It refuses the
current domain and does not accept arbitrary playback flags. Playback waits three
seconds for discovery; start the offline consumer first, especially for short bags.
This delay is not an acknowledgement that every subscriber is ready. Do not run it from
the terminal already set to domain 78. Selecting another domain does not prove
that it is unused: verify no vehicle nodes are running there.

For calibrated input, supply the recorded spectra parameters and preserved
calibration file to the offline spectra process, updating its path to the session
copy. Use the original raw/rectified setting to avoid double correction. Replay
does not inject recorded detections, mission state, motion commands, telemetry,
Action requests or service requests into the live stack.

## Verification

`bash scripts/verify.sh` builds all packages and runs the regression suite.
The qualification test records pass → turn → return, checks nonzero command and
applied output, Action feedback, mission completion, parameter evidence and
command receive gaps. The MAVROS UDP fixture checks raw/normalized telemetry,
actual endpoint overrides, disconnect evidence and rejection of actuator commands.
Dedicated recording tests cover duration flushing, preserved timestamps/config
hashes/calibration, missing sources, disk-reserve stopping, existing output
protection and camera-only replay on a different domain. These are mock and
loopback tests; Pixhawk/ArduSub and onboard-storage commissioning remain pending.
