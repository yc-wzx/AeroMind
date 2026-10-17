# Isolated real sensor-local LIO output phase correction

Original source, install_mid360, its build manifest and all real raw captures are
preserved. This stage copies the exact verified producer into a separate build
directory, changes only output routing and installs to install_mid360_lio_phased.
No estimator, noise, extrinsic, scan, planner or motor parameter changes.

Output contract scan_corrected_primary_v1:
- /hardware/mid360/lio/odometry: scan-corrected states only, nominal 10 Hz.
- /hardware/mid360/lio/odometry_predicted: IMU-predicted states only, nominal 200 Hz.
- /hardware/mid360/lio/odometry_mixed_legacy: EVERY original output, including any
  out-of-order scan correction. No sorting, dropping or retimestamping.
- TF for mid360_local_odom -> mid360_imu: scan corrections only.

Twist is unchanged from upstream and is not a stationarity measurement.
This remains sensor-local, not chassis-centred or a verified navigation source.

1. Keep the real driver running. Compile using:
   BUILD_JOBS=2 bash tools/hardware/mid360_lio_phased/build.sh
2. Stop ONLY the existing LIO terminal (Ctrl+C), leaving raw driver running.
3. Start the isolated output variant:
   bash tools/hardware/mid360_lio_phased/run.sh /path/to/passing/raw/summary.json
4. Keep hardware stationary and collect from another terminal:
   bash tools/hardware/mid360_lio_phased/observe_static.sh --seconds 30

The recorder keeps every received serialized message from all three odometry
channels and TF. Independent audit decodes original CDR and recomputes finite
values, per-phase order, 0.2s continuity gates, phase/mixed identity and TF matching
over their common device-time interval. Boundary records are retained and counted.
Writer counts, source identity, actual parameters and input hashes are gated.
No ground truth or invented drift acceptance limit is used. An evidence_complete
result is not navigation, motor or competition acceptance.

All captures go to new ~/AeroMind_mid360_lio_checks/static_phased_* directories.
Repeat independent audit without overwriting summaries:
   bash tools/hardware/mid360_lio_phased/env.sh python3 -B tools/hardware/mid360_lio_phased/audit.py /path/to/static_phased_capture --output-name reaudit_v1.json

To use the preserved old LIO again, stop only the new LIO and use the original
tools/hardware/mid360_lio/run.sh. Do not run both LIO instances simultaneously.
