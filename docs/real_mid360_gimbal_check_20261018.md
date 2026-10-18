# Real MID-360 manual gimbal turn/return preparation — 2026-10-18

Status: TOOLS READY / PHYSICAL MOTION CHECK PENDING.

## Current accepted evidence

The user-pasted physical stationary report at
`/home/rm/AeroMind_mid360_lio_checks/static_phased_20261017_224645/summary.json`
has evidence_complete=true. Corrected=301 messages (~10 Hz), predicted=6008
(~200 Hz), TF=301 (~10 Hz), all without timestamp regressions or duplicates.
Mixed legacy=6309 messages, retaining 301 regressions for comparison. All 12919
received messages were written; there were no overflow/writer errors. Interior
identity comparison matched corrected + predicted to mixed and matched TF poses.
Native SHA-256: `21bc2fce870199274e7dc8975154c651d45bd303c0f195d1097c766c36052be6`.
This report was supplied by the user; the development machine does not possess
the physical native file and does not claim an independent replay of it.
Estimated 30-second maximum position change was 0.020098567 m; final 0.012533194 m.
Maximum orientation change was 0.735841 degrees; these are not ground-truth errors.
The user confirms the radar is mounted inverted on the yaw-only gimbal and can
be rotated independently with the chassis fixed.

## New independent read-only tool

`tools/hardware/mid360_gimbal_check/` is a sibling of both sealed input stages.
It adds no estimator/planner/control changes and no build step. Existing raw and
phased source/install/manifests remain unchanged (399 and 148 entries verified).

The interactive operator sequence is: initial hold 5s; manual CCW turn about 45
degrees viewed from above; turned hold 5s; manual return to the physical initial
mark; final hold 5s. Motion starts only after Enter, and the operator presses
Enter after stopping. The fixed chassis and planned direction remain human
declarations. The approximate angle target is not used as measured angle truth.
No hardware motor command is sent by this tool.

Every received corrected/predicted/mixed/TF/SI-IMU serialized message is recorded,
with original device fields and wall/monotonic receipt times. A background scoped
ROS executor remains active during stdin waits. Native data and operator markers
are hashed after closing; build and local tool inputs are captured before/after.
Original timestamp order, finite values and existing 0.2s output/0.1s IMU gap
limits are retained. The independent reader validates markers, phase coverage,
identity multisets, source/parameter checks and writer health.

Final-two-second hold medians/quaternion means yield relative turn and return
orientation and sensor-origin closure. The baseline stationary specific-force
axis estimates up for yaw-only sign interpretation. Negative native sensor yaw
can be expected with an inverted mount. This is not measured mounting extrinsics.
Gyro and LIO are not independent references: agreement is internal consistency.
Position movement caused by the sensor pivot lever arm is not called drift or
cancelled using guessed geometry. No accuracy pass threshold is invented.
The five-degree floor only establishes that a rotation was exercised. Even a
motion_check_complete result does not certify angular/position closure accuracy.

## Local verification and limits

Eight targeted tests pass: inverted sign and nonzero pivot displacement, q/-q
and angle wrap, incomplete phase markers, missing IMU, nonfinite IMU, no-motion
NOT_EXERCISED, data-hash tampering and capture-failure preservation on re-audit.

An isolated localhost domain 220 test exercised the actual background recorder,
operator phases and independent reader using an explicitly synthetic mounted
sensor trajectory. Scripted keyboard waits allowed a 45-degree turn and return,
with 0.1m synthetic lever arm and no true translation of the synthetic pivot.
It recorded all 16378 received messages without overflow/error. Estimated signed
rotations were +45/-45 degrees; gyro integrals about +45.0013/-45.0002 degrees.
Sensor outward displacement was 0.0765367m and final closure 0. No physical accuracy
claim follows from this synthetic fixture. Independent re-audit was identical.
The global ROS executor was prohibited during this test; the scoped executor was
used throughout. A separate actual CLI test with no publishers returned exit 2,
zero data and incomplete status, printing prerequisite startup commands.

Evidence: `docs/evidence/mid360_gimbal_20261018/`.
Synthetic capture:
`/home/rmnav/AeroMind_mid360_gimbal_checks/turn_return_20261018_105634/summary.json`.

## Mini-PC operation

Keep the sensors-only driver and phased LIO terminals running. No C++ rebuild or
repeat of the completed stationary capture is required. In a third normal terminal:

```bash
cd ~/AeroMind_20261016
git pull --ff-only
bash tools/hardware/mid360_gimbal_check/run.sh
```

Follow the Chinese prompts; do not rotate before readiness passes. Keep the
chassis fixed and use a visible initial orientation mark. At the end provide the
printed summary. If inputs are missing, the tool prints both prerequisite commands.
Physical turn/return, pivot calibration, hand-pushed translation, chassis pose,
navigation integration and motor integration remain pending.
