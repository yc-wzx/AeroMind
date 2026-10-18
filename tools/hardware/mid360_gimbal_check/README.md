# Mounted inverted MID-360: manual yaw and return diagnostic

Keep the real driver and phased LIO terminals running. No rebuild is needed.
Run from the project root in a normal interactive NoMachine terminal:

    bash tools/hardware/mid360_gimbal_check/run.sh

The tool checks current input identities, finite/fresh data and the phased build.
It then guides one trial: five-second initial hold, manual CCW/left turn about 45
degrees viewed from above, five-second turned hold, manual CW return to the
initial physical mark, and five-second final hold. Press Enter BEFORE beginning
each movement and AFTER stopping it. Chassis must remain fixed. The approximate
45-degree target is not measured angular truth.

Only subscriptions and read-only parameter requests are used. No goals, cmd_vel,
serial, actuator control, clock edits or estimator parameter changes occur.
Native messages are received by an explicit scoped ROS executor in a background
thread, so keyboard waits do not pause capture. A bounded asynchronous writer
retains each received serialized message from corrected, predicted, mixed, TF
and SI IMU, including timestamps and original global receipt sequence.

New records go to ~/AeroMind_mid360_gimbal_checks/turn_return_*/.
Analysis verifies the native hash, marker sequence, writer/source/input gates,
existing per-phase odometry/TF continuity and IMU 0.1s continuity, then reports
rotation direction, measured gyro component integrals and final hold closure.
Hold statistics use the final two seconds of each actual five-second hold; their
median positions and quaternion means are derived statistics, not rewritten raw
samples. Relative rotations are sign-safe and handle angle wrap.

For the inverted sensor, native local Z yaw may be negative for physical CCW.
The initial stationary specific-force direction estimates upward for a yaw-only
direction projection. This is not mounting calibration. IMU and LIO share input;
their agreement checks internal consistency, not independent angular accuracy.
Sensor translation during rotation may arise from the unknown pivot lever arm.
It is not called chassis drift and no guessed lever-arm compensation is applied.

evidence_complete is recording/audit completeness. motion_check_complete only
adds the minimum 5-degree exercise and internal direction checks; it DOES NOT
certify return-position/angle accuracy or navigation readiness. Closure values
are reported without an invented pass threshold. Missing/invalid evidence,
incomplete markers or captured failures cannot become a pass on independent audit.

Independent re-read (use a fresh output name):
    bash tools/hardware/mid360_lio_phased/env.sh python3 -B tools/hardware/mid360_gimbal_check/analyze.py /path/to/capture --output-name review_v1.json

The original 399-entry raw build, 148-entry phased build and all old data stay
unchanged. If inputs are absent, the tool saves an incomplete record and prints
the exact two prerequisite startup commands; do not move during a failed precheck.
