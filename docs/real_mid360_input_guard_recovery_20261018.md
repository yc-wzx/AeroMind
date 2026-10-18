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
Real recovery results and limitations are recorded below.

Read tools/hardware/mid360_lio_guard/README.md for the new optional entry point.
Old raw/phased folders, builds, input manifests and physical records are kept.

On-device format precheck stopped the first guarded session BEFORE launching LIO:
valid 0.10007936s MID360 clouds contained 130 internal time regressions. Upstream
imu_processing.hpp sorts points for deskew. The new strict within-cloud ordering
assumption was removed and a regression test added; counts remain recorded and
raw payloads unchanged. This does not relax span or input-gap limits.

A second short startup guard fault exposed another invalid added assumption:
latest IMU vs scan START differed by 0.200556207s, while the most recent cloud
actually covered 0.100314112s. Cross-stream checks now compare the original IMU
time against the measured cloud time interval, not its start alone. The 0.2s
cross-stream, 0.2s cloud and 0.1s IMU gap limits remain unchanged. New tests cover
this start/end distinction. Both startup records are retained.

## Real-device recovery result

At the user's confirmed stationary condition, only the identified divergent LIO
launch PID23126/child23209 was stopped. Raw driver PID22814 stayed running.
Old final logs and failed hand-push evidence remain preserved.

Active guarded session:
/home/rm/AeroMind_mid360_guard_checks/session_recovery_20261018_163111
Exact estimator binary, configuration and sealed 148-entry input chain remain
unchanged; only guarded input remappings and optional session supervision differ.

| Static capture | Full evidence | Gross-sanity result | Max estimated displacement | Max rotation |
|---|---|---|---|---|
| First 30s after initialization | complete | FAIL | 0.121858m | 2.329740deg |
| One 30s post-warmup check, same session | complete | PASS | 0.039832m | 1.052962deg |

The first result is retained as FAIL. Its first 5s contributed 0.08284m;
subsequent 5s bins were 0.0153..0.0442m. This supports a startup convergence
interpretation, not a proof of the unique estimator cause. No threshold was
changed and this is not statistical reliability validation.

Post-warmup corrected stream: 303 samples, 10.00031Hz, maximum receipt gap
0.102600s, no duplicate or regressed stamps. IMU: 6057 samples, 200.00117Hz,
maximum receipt gap0.006830s. Final 5s max displacement0.023588m.
V1 and V2 independent audits exactly agree. Independent post-capture verification
also checks every original raw hash, individual saved tool snapshot, current
source hashes and actual estimator binary/resolved path/configuration hashes.

Physical raw archive SHA256:
2c415d1eae69b142cc1e21adee2d18c0a255e9085826e445c0fa5f622ea201c3
Windows: artifacts/mid360_guard_recovery_20261018_evidence.tar.gz
The running session logs/status in the archive are explicitly point-in-time
snapshots, not final closed-run hashes. Static captures are closed and hashed.

15 final guard tests and the previous 16 translation tests passed. New validation
uses no wheel odometry, motor publication or auto-aim process. No real movement
retest has been performed after recovery; a new 0.50m trial terminal is prepared
and waits for operator Enter. It rejects a stopped/stale/faulted guard session
before opening the read-only capture. Its baseline 5s pose sanity must pass.

Status: INPUT-GAP FAILURE CONTAINMENT AND STATIC RECOVERY VERIFIED.
Not distance-accuracy, calibrated extrinsics, navigation or competition acceptance.
A fresh physical 0.50m A-B-A check remains the next required step.
