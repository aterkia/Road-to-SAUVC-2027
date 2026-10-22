# Milestone ledger

PRD.md remains the requirements baseline. No hardware, frame conversion to an
autopilot, or 2027 competition rules are inferred from mock results.

The repository initially contained PRD.md and user-deleted legacy orchestrator
files. The deleted mission type implementation is not restored.

## Implementation decisions

- Use the locally available ROS 2 Lyrical / Ubuntu 26.04 container for verification.
  This is a development baseline, not a companion-computer hardware decision.
- Keep the five package boundaries and languages specified in PRD sections 5/9.
- Start in mock mode, idle and unarmed. Reject hardware mode until its adapter is
  implemented and bench-verified.
- Use ordinary functions/state machines; no behavior tree or navigation framework.
- Before multiple tasks in M3, introduce the technically required action interface.
- Gate success in simulation will require measured vehicle displacement and visual
  evidence, not elapsed motion time. Hardware needs a validated measurement source.
- Failures stop the plan; retries require a new operator start unless a future
  validated policy explicitly adds retry behavior.

## Gates

| Stage | State | Evidence / remaining work |
| --- | --- | --- |
| M0 | Passed | `colcon build`: 5 packages; Python entry point test; all four nodes discovered through bootstrap |
| M1 | Mock passed; bench pending | Build + 7 system tests pass; logs `artifacts/m1.log` |
| M2 | Synthetic baseline passed | 4 Python package tests + 9 system tests; `artifacts/m2.log`; pool recording validation pending |
| M3 | Mock passed; hardware pending | 17 system tests through colcon, 0 failures; `artifacts/m3.log`; recorded qualification pass/turn/return |
| M4 | Available subset passed; milestone incomplete | Drum/flare lab baselines + prerequisite tests pass; payload/localization and final rules block full completion |

## Original M0–M4 software gate

`artifacts/m4.log`: all 6 packages built; 18 system tests, 4 C++ prerequisite
tests and 6 Python package tests passed. `colcon test-result` reports 24 results
(including CTest wrapper results), zero errors/failures/skips. Python package
unittest results are printed separately by colcon.

The qualification test now launches the installed bootstrap entry point and
loads YAML; the timeout test replaces the task sequence in YAML without a source
change. The recorded world has a fixed gate independent of mission state, and
the test asserts both outbound and inbound crossing using mock ground truth.
Artifacts are local and gitignored; regenerate them using `scripts/verify.sh`.

Final verification on 22 October 2026:

- Fresh build from the ROS underlay into `/tmp/aterkia-clean-build` and
  `/tmp/aterkia-clean-install`: all 6 packages passed (`artifacts/clean-build.log`).
- Perception-only replay on a separate ROS domain: 233 received frames, 68 gate
  detections and 165 empty frames; source frames preserved (`artifacts/replay.log`).
- Successful compressed recording: `artifacts/qualification-ye_03xpq/bag`.
- `git diff --check` and Python compilation completed without errors.

The local validation image was `ros2-docker-ros2:latest` (`cec14872ff11`), ROS 2
Lyrical / Ubuntu 26.04. The user's existing `ros2-lyrical` container was not
modified; a separate `aterkia-validation` container was used for this session.

## Research adaptations: ideas 1 and 2 (22 October 2026)

Operator scope at that gate: implement idea 1 first, verify it, then idea 2.
Ideas 3–7 were deferred at that point. No new runtime package, detector/model,
SLAM, payload controller or recording feature was added in those two adaptations.

| Stage | Implemented | Verification |
| --- | --- | --- |
| Idea 1 | Offline chessboard calibration with per-view RMS, optional fixed-map rectification, matched CameraInfo and physical bearing in wayfinder | Before idea 2: all 6 packages built; 25 system tests + 4 C++ tests + 6 Python package tests passed (`artifacts/idea1-verification.log`) |
| Idea 2 | Separate MAVROS/helm bench, configurable endpoint/firmware metadata/mode, state/IMU/pressure telemetry, explicitly locked arm/mode/motion interfaces | MAVROS UDP peer checks actual loaded plugins, NED→ENU and hPa→Pa, command rejection and disconnect (`artifacts/idea2-udp.log`) |
| Combined regression | Both adaptations with existing mock missions | All 6 packages built; 33 system tests + 4 C++ tests + 6 Python package tests passed (`artifacts/adaptations-verification.log`); zero errors/failures/skips |

The 33 system tests include 8 calibration tests and 7 MAVROS bench tests.
`colcon test-result` reports 39 results including CTest wrappers; the 6 Python
package unittest cases are printed separately. The final gate also exercises the
complete chessboard CLI and protects existing calibration files against overwrite.
A separate build sourced only from the ROS underlay, with new build/install
directories in `/tmp/aterkia-adaptation-*`, also built all 6 packages successfully
(`artifacts/adaptations-clean-build.log`). Python compilation, local documentation
links and `git diff --check` passed.

Validation uses MAVROS 2.16.0, MAVLink 2026.9.9 and OpenCV 4.10.0 on ROS 2 Lyrical.
The separate validation container was updated with MAVROS, the egm96-5 geoid and
compatible ROS interface/type-support packages; the original image alone no longer
describes its installed dependencies. Selected exact versions are in
`artifacts/adaptations-environment.txt`. The user's `ros2-lyrical` container was
not modified.

The MAVLink peer is a loopback protocol fixture, not ArduSub SITL or hardware.
No real camera/housing calibration, sensor mapping, serial-link verification,
mode/arming acknowledgement, thruster neutral/watchdog or pool commissioning is
claimed. Hardware control remains unavailable even when `control_enabled` is
requested. Pressure-to-depth is disabled until sensor identity, surface pressure
and water density are supplied and verified. The M1/M3 hardware gates and full M4
therefore remain open.

Usage and contracts: [camera calibration](camera-calibration.md),
[MAVROS bench](mavros-bench.md). Failed intermediate runs remain under ignored
`artifacts/`; the named final verification log above is the combined passing run.

## Research adaptation: idea 3 (22 October 2026)

Operator subsequently authorized idea 3. Ideas 4–7 remain deferred. Recording
and restricted replay live in `scripts/` with installed bootstrap executables
and capture/QoS configuration; no new runtime package or vehicle control path.

| Stage | Implemented | Verification |
| --- | --- | --- |
| Mock recording | Camera/perception, command/applied output, odometry, health, mission and navigation Action feedback/status; configuration hashes and live parameter snapshots | Qualification pass → turn → return completes and the MCAP is reopened and inspected |
| Bench recording | Raw/normalized telemetry, observed mode/arming, actual endpoint override and configured firmware metadata | MAVROS loopback fixture records disconnect while actuator requests remain rejected; missing RC feedback is explicit |
| Session lifecycle and replay | Duration/SIGINT flushing, disk-reserve stop, no overwrite, calibration copy, image-only replay on a distinct domain | Four recording tests validate storage contents/timestamps, invalid limits, simulated disk pressure and absence of command publishers during replay |
| Combined regression | Existing mock missions and adaptations 1–3 | `artifacts/idea3-verification.log`: all 6 packages built; 37 system tests, 4 C++ tests and 6 Python package tests passed (47 actual cases) |

`colcon test-result` reports 43 results including CTest wrappers, zero
errors/failures/skips. Python package unittest cases are reported separately.
The integration tests invoke the installed recording/replay tools through
`ros2 run`. The 12 calibration/recording tests also passed in the focused run
`artifacts/idea3-startup-replay-tests.log`. Python compilation, local documentation
links and `git diff --check` passed.

Final session paths and topic counts are in `artifacts/idea3-evidence.json`.
The qualification command receive gap peaked at 0.372 seconds in the combined
run, within the 0.4-second mock test budget but with limited margin. An earlier
isolated qualification run measured 0.051 seconds. These are receive-time
observations in the development container, not onboard timing guarantees;
repeat throughput/deadline measurements with the actual camera and storage.
Selected dependency versions are in `artifacts/idea3-environment.txt`.

Intermediate runs exposed Fast DDS shared-memory errors and test startup races.
Final verification ran sequentially in a freshly restarted validation container.
Mock fixtures now observe several armed telemetry updates before requesting
goals, following the existing sensor-fixture settling approach; production goal
rejection, watchdogs and mission retry behavior were not relaxed. Replay has a
three-second discovery delay, and its test uses an executor for the offline
ROS context. Failed/interrupted intermediate logs remain under `artifacts/`;
the named final log above is the complete passing gate.

No Pixhawk, physical actuator feedback, hardware firmware-version detection or
onboard storage performance is claimed. Configured firmware is explicitly
operator metadata. Usage, timestamps, missing-topic semantics and remaining
limits are documented in [recording and replay](recording.md).

## Workspace build and ABI repair (22 October 2026)

After the earlier validation gates, the operator authorized updating 59 ROS
message/interface/type-support packages in the user's `ros2-lyrical` container.
Versions before and after are saved in `artifacts/workspace-ros-packages-before.tsv`
and `artifacts/workspace-ros-packages-after.tsv`. MAVROS libraries now resolve
their dependencies without the previously missing serialization symbols, and
`rosdep check` reports all system dependencies satisfied.

A clean build from `/home/lain/Dev/projects/Road-to-SAUVC-2027` in that container
built all 6 packages. The complete verification passed: 41 system tests, 4 C++
unit tests and 6 Python package tests (51 actual cases). `colcon test-result`
reports 47 results including CTest wrappers, with zero errors/failures/skips.
The final log is `artifacts/workspace-abi-rebuild-verification.log`.

Old generated outputs are preserved in
`artifacts/pre-abi-rebuild-20261022-065058/`. The workspace cache now references
the operator's actual path. `scripts/verify.sh` rejects caches from another
checkout path and supports `ATERKIA_BUILD_ROOT` to isolate container outputs;
four tests cover output paths, cache rejection and build-failure handling.
The spectra build dependency is `python3-setuptools`; `ament_python` remains
its exported build type. Updated setup steps are in the [README](../README.md).

Verification still uses mock sensors/vehicles and a MAVLink loopback fixture.
The hardware acceptance gates remain pending.

## Confirmed hardware (22 October 2026)

Pixhawk with ArduSub, a depth sensor, and eight thrusters in the operator's
Vectored-6DoF layout (see [motor numbering](hardware-and-finals.md)).
No additional sensor or ball-drop/pickup
hardware is available. A measured horizontal displacement source and gate-crossing
criterion remain unverified on hardware. Mock odometry is simulation ground truth,
not an estimate derived from the available depth sensor.

## Explicit arming/disarming (22 October 2026)

`bootstrap arming.launch.py` enables `/vehicle/arm` through the MAVROS command
plugin. The existing telemetry bench remains read-only. Success requires a
positive MAVROS result and fresh FCU state matching the requested arm/disarm.
There is no startup arming, force-arm, automatic retry or rearm on reconnection.
Timeouts/disconnect report an unconfirmed outcome, not a successful cancellation.
Motion, mode changes and hardware mission execution remain unavailable.
See [usage and limitations](arming.md).

Eight new system tests cover explicit requests and state confirmation, FCU
rejection, ACK without state change, unavailable service, missing/late ACK,
stale IMU with disarm still allowed, disconnect, overlapping requests, and the
real MAVROS transport. The loopback peer verifies command 400 with normal
arm/disarm values and force value zero; motion/RC packets remain absent while
`/cmd_vel` is published. This peer is not ArduSub SITL or a dynamics simulation.

All 6 packages built in the isolated validation container. Final full verification
passed 49 system tests, 4 C++ unit tests and 6 Python package tests (59 actual
cases). Colcon reports 55 results including the CTest wrappers. Evidence:
`artifacts/arming-build.log` and `artifacts/arming-final-verification.log`.
`helm` and `bootstrap` were then rebuilt in the operator's `ros2-lyrical`
workspace, and the installed arming launch was checked with `--show-args`;
see `artifacts/arming-workspace-build.log`. No hardware arming command was sent.

The recording fixture now waits for the IMU subscription and records a sustained
sensor stream before disconnecting the peer; discovering a DDS endpoint alone
does not establish that its TCP data stream has started. Two existing mock
guidance/vision tests failed in an intermediate run, passed separately, and passed
in the final full run; their production code and acceptance checks were unchanged.
Intermediate failures are retained in `artifacts/arming-targeted-tests.log`,
`artifacts/arming-targeted-recheck.log`, `artifacts/arming-full-verification.log`
and `artifacts/arming-full-recheck.log`.

Real Pixhawk arming/disarming and physical thruster output have not been tested.
The diagram is a confirmed intended layout; wiring and motor directions still
need physical verification. No autopilot frame/output parameters were changed.
