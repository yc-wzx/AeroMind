# Corrected sensor-local LIO stage (no raw capture mutations)

The older preparatory YAML incorrectly used physical frame name mid360_imu as
the SPARK visualization mode. The actual output code only accepts imu/lidar/base.
This separate stage uses corrected mode imu and keeps physical frame mid360_imu.
Old raw capture tools, input manifest and sealed records remain byte-identical.
No simulation settings, planner, motor commands or navigation frames are changed.

Keep sensors-only driver running. Update via git pull; no C++ rebuild required.
Run: bash tools/hardware/mid360_lio/run.sh /path/to/raw_capture/summary.json
It verifies the original manifest/data hashes, reports clock epochs and unmatched
raw/SI records, checks actual live sources, then starts ONLY the corrected LIO node.
Do not start the old optional lio:=true entry. Stop only this LIO terminal with Ctrl+C.

After initialization, keep device/gimbal/chassis stationary and run the separate
observe_static.sh --seconds 30 from another terminal. Evidence completeness and
relative estimated drift are reported; no ground-truth or navigation acceptance.
The raw sensor-local map is not yet a gravity-aligned/chassis-centred arena map.

The startup precheck also owns a SingleThreadedExecutor bound to its scoped ROS
Context. It does not use the uninitialized default/global executor. Startup
readiness thresholds and the raw build manifest remain unchanged.
The Context regression tests use synthetic sensor messages only in a separate
local ROS domain; do not run them in the hardware domain:
MID360_ROS_DOMAIN_ID=217 bash tools/hardware/mid360/env.sh python3 -B tools/hardware/mid360_lio/test_start_context.py
