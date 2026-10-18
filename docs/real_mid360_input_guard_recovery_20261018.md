# MID-360 LIO interruption recovery fix (2026-10-18)

The 16:10 physical hand-push trial remains a localization failure. Its intact
native messages were independently re-audited: LIO was already diverged before
manual motion. The new reviewer rejects its pose health without changing raw
records or interpreting the huge coordinates as a calibration result.

Confirmed source defects: malformed scan durations were warned but accepted
into scan timing; IMU prediction only checked positive dt, not a large gap.
The uncaptured interruption does not establish a unique historical cause.

This isolated session adds input validation, latched continuity faults and
supervised termination; it reuses the exact sealed phased estimator/config.
No filter mathematics, noise, extrinsics, planner or motor outputs are changed.
Translation recording now includes live gross-pose sanity and all 5s stationary
holds. Independent review reconstructs these checks from original CDR.

Engineering gates (NOT accuracy acceptance): input receive/device gaps <=0.2s
LiDAR and <=0.1s IMU; cloud span <=0.2s; slow manual-test pose speed <=3m/s;
stationary displacement <=0.10m and rotation <=10deg over the declared hold.
No timestamp editing, automatic restart or fitting to measured 0.50m occurs.

Validation: 13 guard tests and 16 prior translation tests passed. A first virtual
DDS/PTTY capture recorded a real 0.2599s final receive gap and was rejected;
that result is preserved. A subsequent isolated virtual +0.50/-0.50m trial passed
with unchanged thresholds and exact independent re-audit. The physical failed
trial now returns LOCALIZATION_SANITY_REJECTED.

Tests and raw synthetic results: tools/results/mid360_guard_fix_20261018/.
Real static recovery results will be appended after the on-device check.

Read tools/hardware/mid360_lio_guard/README.md for the new optional entry point.
Old raw/phased folders, builds, input manifests and physical records are kept.
