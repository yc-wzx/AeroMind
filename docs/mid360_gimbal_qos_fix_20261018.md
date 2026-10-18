# MID-360 mounted gimbal recorder QoS fix — 2026-10-18

## Outcome and scope

Local fix and regression complete. Physical mounted gimbal turn/return remains pending.
Only the read-only observer and its tests changed. No sensor, LIO, navigation,
motor output, timestamp, safety threshold or estimation parameter was changed.
No rebuild or driver/LIO restart is required.

## User-observed failure (summary supplied by operator)

`/home/rm/AeroMind_mid360_gimbal_checks/turn_return_20261018_114352/`
reported corrected=80, predicted=1603, mixed=1683, tf=80, imu_si=0;
ROS explicitly reported incompatible RELIABILITY on `/hardware/mid360/imu_si`.
The real SI adapter publishes `qos_profile_sensor_data` (BEST_EFFORT), while
the observer incorrectly requested RELIABLE for all five streams.
The 3447 received records were preserved. There were no operator phase markers,
so this is incomplete capture evidence, not a failed motion-accuracy result.
The original physical JSONL was not available locally for independent review.

## Minimal change

`tools/hardware/mid360_gimbal_check/record.py` now selects BEST_EFFORT for
`imu_si`, preserving RELIABLE for corrected/predicted/mixed/TF. Depth remains
1000; receive timestamps, raw CDR capture and all audit thresholds are unchanged.
Capture metadata now records requested and discovered offered QoS. A readiness
failure also saves the discovered publishers instead of leaving them empty.
Missing, stale or nonfinite IMU still prevents readiness; no data are fabricated.

## Regression and evidence

Evidence: `docs/evidence/mid360_gimbal_qos_fix_20261018/`.
Local result directory: `tools/results/mid360_gimbal_qos_fix_20261018/`.

1. Before editing, executed the actual production recorder against an isolated
   localhost-domain-220 source with BEST_EFFORT IMU and RELIABLE LIO/TF. The same
   RELIABILITY warning and imu_si=0 occurred; other streams were live. Saved in
   `reproduction.json`, `capture_before.txt`.
2. After the fix, executed production `record.main()` with only keyboard actions
   scripted, using the same BEST_EFFORT source. Recorded 16361 native messages,
   no overflow/write errors; all evidence gates passed. Closed-file independent
   re-audit was identical. Confirmed the actual discovered IMU offer and requested
   reader are BEST_EFFORT; the four other readers/offers remain RELIABLE.
3. Twelve tests pass, including real QoS policy compatibility, missing/stale/NaN
   readiness rejection and the prior offline motion/evidence tests.
4. Sealed phased manifest verifies 148 entries (including raw verification);
   no files in the sealed raw or phased directories were edited.

The previous synthetic fixture incorrectly published RELIABLE IMU and therefore
missed this physical interoperability bug. Its old results are retained, but do
not establish BEST_EFFORT interoperability. This version fixes the fixture.
Two intermediate assertion failures in the new test were retained: Humble returns
a tuple, and the compatibility helper warns about SYSTEM_DEFAULT liveliness.
The final assertion rejects ERROR; actual DDS message delivery is separately
proven by the end-to-end run. Only tests.py changed after that successful run;
the production recorder hash remains the one captured for that run.

The synthetic motion (+45/-45 degrees with an inverted frame and 0.1 m lever arm)
is a deterministic kinematic fixture, not a real sensor or LIO estimate. It proves
observer/auditor operation under real DDS QoS, not hardware angle accuracy,
lever-arm calibration, motor integration or navigation acceptance.

## Reproduce locally

From the development checkout with sealed raw/phased installs:

```bash
bash tools/hardware/mid360_lio_phased/env.sh python3 -B tools/hardware/mid360_gimbal_check/tests.py
python3 -B tools/hardware/mid360_lio_phased/variant.py verify
```

The archived e2e runner is tied to the local evidence paths and intentionally
uses exclusive output files; do not overwrite an existing run. Its control
fixture requires isolated localhost ROS domain 220. It sends no hardware commands.

## Mini-PC next action

Keep the existing raw-driver and phased-LIO terminals running. In a third
ordinary NoMachine terminal:

```bash
cd ~/AeroMind_20261016
git pull --ff-only
bash tools/hardware/mid360_gimbal_check/run.sh
```

Wait for 已就绪, then follow its Enter prompts. Keep chassis fixed throughout;
turn gimbal slowly CCW viewed from above, hold, return to the physical start mark,
hold. Do not use the keyboard-only fixture on hardware. Review the new summary;
the old failed directory is retained unchanged.
