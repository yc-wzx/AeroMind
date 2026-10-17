# Real sensor-local LIO timing review — 2026-10-17

Status: LIO RUNNING / OUTPUT TIMESTAMP ORDERING BLOCKED.
The raw input capture remains passed. The previous startup Context bug is fixed.
The new physical stationary capture is NOT an accepted navigation output.

## User-supplied physical result

The mini-PC report at
`/home/rm/AeroMind_mid360_lio_checks/static_20261017_221527/summary.json`
records 6301 valid odometry messages across 30.001 seconds (~209.993 Hz),
300 adjacent sensor timestamp regressions, no duplicate stamps, no invalid rows,
maximum receive gap 0.019189316 s and matching received/written/line counts.
Its unique publisher is the expected `mid360_real_lio`, with corrected `imu`
visualization mode, sensor frame `mid360_imu`, and `use_sim_time=false`.
These are user-pasted summary facts; the native JSONL has not yet been transferred
to the development computer or independently audited here.

Under the declared stationary condition, estimated maximum relative translation
was 0.0190934211 m, ending at 0.0131907526 m. Estimated maximum orientation change
was 0.562692 degrees, ending at 0.095176 degrees. These are sensor-pose changes,
not ground-truth errors. No drift acceptance threshold or navigation pass is inferred.

## Verified source mechanism; per-message attribution still pending

The unchanged upstream producer has two calls to its same publishOdometry output:

- `SPARKFastLIO2::integrateIMU` publishes a predicted state at the IMU stamp
  (`spark_fast_lio.cpp`, approximately line 495).
- The scan update publishes corrected `latest_state_` at `lidar_end_time_`
  (approximately line 1155).

They share the same odometry publisher, parent/child frames and TF output. The
source therefore can emit a scan correction older than a previously published
prediction. The observed 210 Hz total and 10 regressions per second are consistent
with 200 Hz IMU prediction + 10 Hz scan correction. This establishes a source
mechanism and a strong timing hypothesis; it does not identify the callsite of
each received record, because the existing Odometry messages have no phase ID.
Also, current publishOdometry does not fill Twist, so zero recorded Twist is not
independent evidence of zero physical motion.

Do not sort records, discard delayed corrections, rewrite timestamps or relax
the strict ordering gate to turn this capture into a pass. Do not yet wire this
mixed output into navigation freshness or control.

## Read-only next step

The new inspect_timing.py / inspect_timing.sh analyze the sealed existing JSONL.
They verify its original SHA-256, validate every row, recompute regressions,
duplicates, receipt gaps, backward jump magnitudes, event spacing and pose jumps,
and recheck original summary/writer consistency. Raw receipt order is preserved.
They also recompute relative pose changes and report the current source hash
against the original capture build manifest, when it is available locally.
Every regression detail is saved in a new timestamped subdirectory. Neither the
old summary nor its JSONL is written. The old incomplete verdict is retained.

Run on the mini-PC after git pull, keeping the current sensors/LIO running:

```bash
bash tools/hardware/mid360_lio/inspect_timing.sh \
  /home/rm/AeroMind_mid360_lio_checks/static_20261017_221527/summary.json
```

This needs no ROS messages, recapture, driver restart, parameter change or C++
rebuild. After raw timing is reviewed, decide whether output phases need explicit
separation. That producer change is not implemented in this round.

Seven targeted offline tests cover mixed-phase regressions without reordering,
duplicate timestamps, invalid/missing/nonfinite data and receipt regressions,
raw hash tampering, inconsistent summaries/health, original-file preservation,
and preventing zero Twist from becoming acceptance. They pass. The unchanged
raw build still verifies all 399 local source/install/SDK entries.
See `docs/evidence/mid360_lio_timing_20261017/` for test output and provenance.

No planner, localization parameters, estimator, navigation output, motor bridge,
raw input implementation or acceptance threshold changed. Physical time-order
diagnosis remains pending the mini-PC command output; no navigation test occurred.
