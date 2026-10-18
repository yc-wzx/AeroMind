# Mounted MID-360 turn/return timing investigation — 2026-10-18

## Operator confirmation and physical result

The operator confirmed: chassis remained on its support without moving, and the
gimbal was rotated by hand. This is consistent with the requested yaw-only test.
Manual operation itself is not a failure reason. There is no evidence to assert
that the whole chassis was lifted or carried. Marker/motion timing is not yet
independently established.

User-pasted report: `/home/rm/AeroMind_mid360_gimbal_checks/turn_return_20261018_120354/`.
Source text and physical raw files are on the operator's PC, not available for
local independent raw replay. A labelled copy of the pasted summary is archived.

The reported capture evidence is complete, but motion_check_complete=false:

| Quantity | Outward | Return |
|---|---:|---:|
| LIO hold-to-hold projected angle | +47.54769 deg | -45.06168 deg |
| IMU integral over operator-marked movement | -0.46785 deg | -66.21136 deg |

Final estimated orientation closure is 2.48984 deg; sensor-origin position closure
is 0.0107802 m. These are reported estimates without independent truth or an
accuracy acceptance threshold. Sensor displacement may include pivot lever arm.

Corrected/TF ~10 Hz, predicted and SI IMU ~200 Hz, no corrected/predicted/IMU
duplicate or backwards timestamps, 65923 received/written lines, no overflow or
writer failure. Source, parameter and unchanged-input gates pass in the supplied
summary. Mixed legacy's 1046 backwards stamps are the expected output contract;
they are diagnostic and do not invalidate the split streams.

This establishes the previous IMU QoS blocker is absent in this capture. It does
not explain the angular inconsistency or prove physical accuracy. Do not change
IMU signs, mounting transforms, units or LIO parameters based only on this report.

## Read-only diagnostic added

`tools/hardware/mid360_gimbal_check/diagnose_timing.py`:

- Reuses the unchanged original audit, including original hashes and gates.
- Re-decodes every recorded corrected/IMU sample, retaining original sequence,
  device timestamps, wall and monotonic receive timestamps.
- Includes all five marked phases, all unmarked gaps, pre/post capture samples;
  checks these windows partition all decoded samples exactly once.
- Computes body gyro component integrals, estimated-up net/travel projections,
  sample coverage and peak rate, plus actual first-to-last LIO angle per window.
- Adds 1-second receive-time bins and lists the strongest IMU/LIO bins. Reports
  wall-minus-device offsets without claiming they are one-way latencies.
- Writes a new report exclusively; does not change markers, interpolate, subtract
  bias, publish ROS data or update acceptance booleans. Null means insufficient
  samples, never a measured zero.

This is a timing investigation, not a generic 3D quaternion integrator. Fixed-up
gyro projections rely on nearly yaw-only motion. LIO/IMU share input and are not
independent truth. First/last segment samples differ from hold-window averages;
unequal sample coverage and receive/device alignment remain explicit limitations.

## Tests and preserved scope

Five targeted tests and the existing twelve tests pass. A misplaced-marker
synthetic case shows >43 deg in the unmarked gap while preserving the original
motion failure. CLI replay also verifies hash preservation and rejection of a
second write to the same report. A previous 16361-message synthetic DDS capture
is analyzed offline; no new Gazebo, hardware or motion test was run.

Evidence: `docs/evidence/mid360_gimbal_timing_review_20261018/`.
Local results: `tools/results/mid360_gimbal_timing_review_20261018/`.
Sealed phased manifest (148 inputs, including raw build verification) still passes.
No production recorder, prior analyzer, raw/phased source/install, navigation,
control, parameter, network or hardware output file was changed.

## Operator's next step: old files only

```bash
cd ~/AeroMind_20261016
git pull --ff-only
MID360_ROS_DOMAIN_ID=43 bash tools/hardware/mid360_lio_phased/env.sh \
  python3 -B tools/hardware/mid360_gimbal_check/diagnose_timing.py \
  /home/rm/AeroMind_mid360_gimbal_checks/turn_return_20261018_120354
```

No sensor processes need to be running for this offline command. Do not perform
another turn yet. It adds `timing_review_v1.json` next to the original data, never
overwriting the original summary/native/events. Return the compact console output
or the new full report. If the output name already exists, preserve it and inspect
it instead of deleting or overwriting it.

Physical root cause: UNDETERMINED. Turn/return motion check: NOT COMPLETE.
TRAINING-ONLY / PROVISIONAL; no navigation acceptance or new motion authorization.
