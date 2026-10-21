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
| `/vehicle/arm` | SetBool; explicit operator authorization; false always stops/disarms |
| `/safety/abort` | Trigger; direct helm stop/disarm, independent of mission processing |

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
