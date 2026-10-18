# Real MID-360 input continuity guard

This optional session preserves sealed raw/phased source, installs, configuration
and historical trials. It forwards validated PointCloud2 and Imu unchanged:
no edits to payload, timestamps, units, frames or configured publishing rate.
The existing phased estimator uses checked input remappings.

Checks: cloud schema, finite fields, point time span <=0.2s, per-point time
interleaving diagnostics, usable returns and exact source identity. Receive AND device time
gap limits: points 0.2s, IMU 0.1s. Faults latch; the supervisor stops only its own
LIO/guard children. No automatic restart, reset, time repair or gap integration.
The raw driver keeps running.

After stopping the old LIO, leaving the driver running:

    bash tools/hardware/mid360_lio_guard/run.sh /path/to/passing/raw/summary.json

Guard publishers contain ONLY checked sensors, no velocity, goal, body odometry
or TF. The unchanged LIO publishes its original sensor-local odometry and TF,
which are NOT qualified navigation outputs.

PoseHealth is catastrophic-failure screening, NOT distance accuracy acceptance.
The 5s stationary engineering gate uses 0.10m displacement from its first sample
and 10deg rotation; gross motion is limited to 3m/s for this slow manual test.
Passing does not certify precision, extrinsics, map alignment or motors.
