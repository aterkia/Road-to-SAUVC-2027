# Hardware handoff and M4 limits

Confirmed by the operator: Pixhawk with ArduSub, a depth sensor, no additional
sensor or ball-drop/pickup hardware. PRD section 13 remains the decision register;
this implementation does not change the competition requirement baseline.

## Available now

- Qualification and Navigation use the tested mock gate action. `NAVIGATION`
  explicitly names the finals prerequisite and maps to the same gate guidance.
- Optional `spectra.enable_finals: true` detects a blue circular drum and red,
  green and yellow tall flares on controlled fixtures. These are configurable
  **lab color defaults**, not claims about SAUVC target colors or underwater
  robustness. HSV bounds, minimum area and image size are YAML parameters.
- Class labels are `drum`, `flare_red`, `flare_green`, `flare_yellow`. Headers
  retain source time/frame; confidence is shape quality, not probability.
- Mission plan validation enforces the PRD's provisional 2026 dependencies:
  finals need `NAVIGATION`; `TARGET_REACQUISITION` needs `TARGET_ACQUISITION`
  and a subsequent `RETURN_GATE`. Execution only advances after successful
  action results, so an earlier failure cannot satisfy a later prerequisite.
- Plans containing payload/localization tasks are rejected at startup, even if
  their ordering is valid. No dummy success or unverified actuator output exists.

## Acceptance scenarios

| Capability | Reproducible acceptance now | Remaining hardware/field evidence |
| --- | --- | --- |
| Drum | Synthetic blue disk produces `drum`, center within 1 px; tall blue distractor rejected | Annotated pool recording, camera calibration, actual target appearance |
| Flare | Three colored vertical fixtures produce matching labels; horizontal distractor/blank rejected | Actual colors, occlusion/turbidity dataset, navigation collision margins |
| Dependencies | Invalid ordering/unknown tasks rejected by C++ tests; valid provisional order accepted by validator | Review against final 2027 rules |
| Acquisition/drop | Unsupported task rejected, no actuator output | Payload geometry/driver, target-relative pose, drop confirmation sensor, authorized command |
| Reacquisition/pickup | Missing acquisition/return gate rejected; execution unavailable | Pickup mechanism, capture confirmation, return localization evidence |
| Communication/localization | Navigation prerequisite validated; execution unavailable | Communication interface, input sequence validation, target localization |

Full M4 is not complete. Mock perception tests cannot close the final-task
acceptance gate without these decisions and measured results.

## Pixhawk / ArduSub integration gate

The [telemetry-only MAVROS bench](mavros-bench.md) now provides connection,
state, IMU/heading and pressure observation. Pressure-derived depth is opt-in
after sensor identity and surface calibration are verified. Endpoint, firmware
metadata and intended mode are parameters. All actuator interfaces remain locked;
software/UDP tests do not close the hardware commissioning gates below.

1. Select and pin the MAVROS/ArduSub integration, firmware version and vehicle
   control mode; use the existing bridge rather than implementing MAVLink.
2. Define depth/heading or velocity setpoints, arming/disarming and neutral/hold
   semantics for that mode. Confirm ROS ENU/FLU versus autopilot frame conversion
   exactly once in the adapter; verify direction and units on a thruster-disabled
   bench before any propulsion test.
3. Confirm camera intrinsics/extrinsics, orientation telemetry and a validated
   means of proving horizontal gate passage. Depth alone cannot establish it.
   Mock `/vehicle/odometry` must never be substituted as hardware evidence.
4. Verify manual override, heartbeat/command loss, telemetry loss and physical
   kill switch independently of mission processing. Retain an autopilot-level
   failsafe if the companion or adapter process dies.
5. Document a controlled hardware session with logs, then agree quantitative pool
   thresholds from the measured sensor and vehicle performance as required by PRD.

`mode:=hardware` remains rejected until this gate is implemented and tested.
