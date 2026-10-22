# Runtime contracts

All nodes use the root namespace. Mock time follows ROS system time for stamps;
watchdogs and task deadlines use a monotonic clock so pausing/jumping ROS time
does not extend motion. `use_sim_time` requires a live `/clock` source and is not
enabled in the kinematic mock.

For 640×480 BGR images the tested Fast DDS transport is `LARGE_DATA` with 1 MB
maximum messages and 2 MB socket buffers. Source `scripts/ros_env.sh` in camera,
test and replay terminals; bootstrap applies the same defaults to its nodes.
The defaults preserve explicitly provided environment overrides. An initial
recording showed >1 s gaps in camera delivery despite 50 ms odometry updates;
its default transport could not sustain the large samples. This configuration
follows [Fast DDS large-data guidance](https://fast-dds.docs.eprosima.com/en/3.2.x/fastdds/use_cases/tcp/tcp_large_data_with_options.html).
Changing image resolution requires rechecking payload size, buffers and latency.

| Interface | Contract |
| --- | --- |
| `/cmd_vel` | Twist; FLU body, surge m/s, yaw rad/s, positive yaw counterclockwise; reliable keep-last 1, volatile |
| `/vehicle/applied_cmd` | Mock adapter output after arming, validation, limits and watchdog; same units/QoS |
| `/vehicle/odometry` | Odometry; ENU `odom`, FLU `base_link`; sensor-data QoS; mock ground truth only |
| `/vehicle/depth` | Float64, metres positive down; sensor-data QoS; mock fixed depth 1 m |
| `/vehicle/heading` | Float64, ENU yaw radians, zero east, counterclockwise positive; sensor-data QoS |
| `/vehicle/health` | DiagnosticArray; reliable keep-last 1; `helm`: OK/ARMED, WARN/DISARMED, ERROR/DISCONNECTED |
| `/vehicle/arm` | Mock SetBool; explicit operator authorization; false always stops/disarms in mock |
| `/safety/abort` | Mock Trigger; direct helm stop/disarm, independent of mission processing |

The table describes the mission/mock launch. The separate [MAVROS bench](mavros-bench.md)
publishes telemetry and WARN/ERROR health, never `ARMED` authorization or
odometry. Its arm/mode/abort services explicitly return failure because no
hardware commands are enabled; it cannot claim a successful hardware stop.

Startup, disarm, abort, command timeout, invalid command and connection/telemetry
loss all produce zero surge/yaw. A new explicit arm is needed after any disarm.
The mock rejects unsupported axes instead of silently ignoring them. These zero
commands are a mock safety policy, not an assertion about neutral setpoints,
buoyancy, or arming semantics on ArduSub.

`wayfinder` rejects wrong odometry frames, invalid quaternions/positions, future
stamps beyond 50 ms and stale stamps. Both source-stamp age and monotonic time
since reception constrain freshness. Headerless depth/heading are informational;
guidance uses stamped odometry instead of treating an unstamped old value as new.

Parameters used for control configuration are read-only after startup. Edit YAML
and relaunch. `helm.mock_connected` and `helm.mock_telemetry` are deliberately
mutable fault-injection controls for tests.

## Visual gate baseline

`/camera/image_raw` accepts timestamped `bgr8` or `rgb8` images in
`camera_optical_frame` using sensor-data QoS. `/perception/detections` preserves
that header and publishes `vision_msgs/Detection2DArray` with class `gate`, pixel
bounding boxes and a geometric quality score in [0, 1]. It is not a calibrated
probability. A fresh empty array means no gate; no messages means invalid/stale
input. Guidance checks source timestamps and receive-time freshness independently.

Optional [calibration](camera-calibration.md) makes boxes refer to rectified
pixels and publishes `/perception/camera_info` (sensor_msgs/CameraInfo, sensor
QoS) with exactly the same source header. With `use_camera_info: true`, guidance
requires a valid exact-time pair and computes `atan2(u-cx, fx)` from P. Invalid
or missing calibration does not fall back to normalized-pixel guidance. Mock
defaults retain the explicit uncalibrated mode.

The baseline finds two similarly sized, vertically overlapping orange posts.
HSV bounds, post shape/area and guidance thresholds live in YAML. The configured
guidance image dimensions must match the camera. Pool recordings and camera
calibration are needed before treating this as a deployable underwater detector.

Search rotates while fresh images show no gate. Align rotates towards the target
(image-right maps to negative FLU yaw). Approach adds forward motion once aligned.
Pass begins after the aligned gate reaches the configured apparent width; its
success requires forward displacement in mock odometry plus three fresh empty
images. Heading/lateral drift or excessive displacement without confirmation
fails the task. Elapsed time is only a failure limit, never passage evidence.

## Mission execution

`/navigation/task` uses `aterkia_interfaces/action/NavigateTask`: a task name,
turn angle and finite positive deadline; phase/progress feedback; terminal
`COMPLETED`, `FAILED` or `ABORTED` status with a reason. ROS action UUIDs identify
each attempt. Only one goal is accepted at a time; cancel publishes a zero
command and terminates the goal. Legacy `/mission/task` String commands are
removed; `/navigation/status` String is diagnostic output only.

`/mission/start` is a Trigger service that requires a fresh armed vehicle,
available action server and no unresolved previous goal. `/mission/abort`
cancels navigation and invokes the direct helm stop. `/mission/status` is a
reliable transient-local String stream for lifecycle and per-task completion.
The orchestrator waits for a successful action result before dispatching the
next configured task and disarms on completion, failure or abort.

Arming alone never starts a task. Reconnection does not resume either a goal or
a mission. Startup, abort and failure handling are tested independently of the
available hardware, whose commissioning remains a separate acceptance gate.

## Recording evidence

[Session recording](recording.md) subscribes to existing interfaces without
vehicle commands. Mock recordings include `/cmd_vel` and `/vehicle/applied_cmd`;
bench recordings include raw/normalized telemetry and observed arming/mode.
Unavailable feedback is listed explicitly. Camera/perception source stamps are
preserved; unstamped commands/status use bag receive times. Perception replay
publishes camera input on a separate domain and excludes recorded control output.

## Onboard ArduSub adapter (autonomous proposal, stages 0–1)

The selected transport is MAVROS `MANUAL_CONTROL` with ArduSub `ALT_HOLD`.
Commands come from onboard guidance. The existing mock `/cmd_vel` contract
retains its SI units and never drives the hardware adapter.

| Interface | Contract |
| --- | --- |
| `/vehicle/command` | `aterkia_interfaces/VehicleCommand`; source Header in FLU `base_link`; dimensionless `surge_input`, `yaw_input`, `heave_input`, each finite in `[-1,1]`; positive forward, counterclockwise yaw, up; reliable keep-last 1, volatile |
| `/vehicle/depth_stamped` | `aterkia_interfaces/DepthStamped`; calibrated metres positive down; original pressure source Header; sensor-data QoS |
| `/vehicle/set_mode` | Trigger; prepare the configured `ALT_HOLD` mode while disarmed; success requires a fresh observed mode after the MAVROS response |
| `/vehicle/arm` | SetBool; autonomous adapter requires successful preparation, verified FCU configuration and fresh IMU/depth; accepted ACK plus fresh observed armed state opens one command session |
| `/safety/abort` | Trigger; cancel preparation/session, send neutral once if active, request disarm; response confirms local cancellation, **not** physical stopping or confirmed disarm |

The adapter starts disarmed and sends no pilot inputs until its own preparation
and arming sequence succeeds. External arming does not authorize commands. The
first command must arrive within `command_timeout`, just like subsequent commands.
Source timestamps must increase, remain within the timeout, and not exceed ROS
time by more than 50 ms. Receipt-time watchdogs use a monotonic clock. Conflicting
publishers, invalid values/frames, telemetry loss, unexpected mode, disconnect,
or timeout latch the session off. Reconnect and late service replies cannot
reopen it. Starting again requires deliberate preparation while disarmed.

The combined absolute input is limited to `input_limit` (initial software-test
value 0.2). MAVROS sends `x=1000*surge`, `y=0`, `z=500+500*heave`,
`r=-1000*yaw`; buttons and extensions remain zero. Neutral is `(0,0,500,0)`.
This is neither a measured motor-power percentage nor a motor PWM command.
ArduSub retains stabilization, vertical control, and the eight-thruster mixer.
See [ArduSub pilot input semantics](https://ardupilot.org/sub/docs/pilot-control.html)
and [MAVROS send mapping](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/manual_control.cpp).

When a session ends, neutral is sent once and disarm is requested. A continuous
neutral stream must not hide a dead guidance process from FCU pilot-input failsafe.
Health reports `ARMED` at level OK only for the locally authorized session;
`preflight_ready` separately describes readiness before arming. Disarm success
requires fresh FCU state; loss of connection leaves the result unconfirmed.

Control configuration is read-only after startup. Enabling the adapter requires
an explicit ArduSub version, `ALT_HOLD`, calibrated water-pressure source, and
arming enabled. The MAVROS parameter cache must confirm Vectored-6DoF frame 2,
matching sender system ID, pilot-input failsafe action disarm (2), timeout >0 and
<=2 s, and RC throttle failsafe disabled when that parameter exists. Old/new
firmware aliases for sender/frame are accepted. These checks read parameters;
they never configure the FCU. Physical calibration and wiring are separate checks.

Telemetry-only and arming-only bench profiles keep their previous behavior and
do not load the motion plugin. The [autonomous launch](autonomous.md) combines
the adapter with onboard guidance and mission countdown. Enabling the adapter
alone does not start a mission.

## Onboard mission and guidance

`auv.launch.py mode:=hardware` loads `autonomous.yaml` over the bench telemetry
configuration. Countdown starts once, after mission configuration is loaded,
using a monotonic clock. Failure/abort/completion does not restart it; service
`/mission/start` cannot bypass it. `/navigation/ready` checks readiness in
wayfinder itself before mode and arming requests. It requires fresh IMU/depth,
verified vehicle preflight, and matched calibrated camera input when the plan
contains `APPROACH_GATE`. The mock query requires armed odometry and camera input.

| Hardware task | Feedback and completion |
| --- | --- |
| `DIVE_TO_DEPTH` | Positive-down stamped depth against the configured target; bounded P guidance with a deadband; tolerance sustained over at least three new samples and both source-time and monotonic settle windows |
| `TURN` | Wrapped ENU IMU heading error, with continuous depth guidance; heading and depth must settle |
| `APPROACH_GATE` | Exact-time calibrated detections; search/align/approach; surge stops at the near threshold, then waits for at least three new confirmation frames over the settle window |

Hardware guidance never subscribes to mock odometry and never interprets SI
`/cmd_vel` as input power. Terminal Action results cause mission disarm; final
completion requires confirmed disarm. Command/telemetry/camera loss fails the
task and latches the vehicle session. `PASS_GATE` remains rejected on hardware
until independent crossing evidence is validated. Approach completion is not
navigation/qualification completion and does not satisfy finals prerequisites.
