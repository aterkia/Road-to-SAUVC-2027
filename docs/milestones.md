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

## Latest software gate

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

## Confirmed hardware (22 October 2026)

Pixhawk with ArduSub and a depth sensor. No additional sensor or ball-drop/pickup
hardware is available. A measured horizontal displacement source and gate-crossing
criterion remain unverified on hardware. Mock odometry is simulation ground truth,
not an estimate derived from the available depth sensor.
