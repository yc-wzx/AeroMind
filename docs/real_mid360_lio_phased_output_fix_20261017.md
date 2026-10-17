# Real MID-360 output-phase correction — 2026-10-17

Status: IMPLEMENTED AND LOCALLY VERIFIED / PHYSICAL RECAPTURE PENDING.
No real navigation, motor integration or chassis-centre localization acceptance.

## Physical evidence and diagnosis

The user's native timing audit preserved the original file hash
`e64f758090f6689cc6734682ebb91460a42bf4ee2e04edf5a119fd84386348bb`.
It found 300 adjacent timestamp regressions in 6301 messages over 30.001 s.
Regression spacing median 0.099988951 s; backward jump median 0.004523442 s,
maximum 0.00592086 s. Corresponding estimated pose step median 0.005052511 m,
maximum 0.012575986 m; orientation step maximum 0.747371 degrees.
Raw capture hash, original build manifest and producer source hashes matched on
the mini-PC. Its original incomplete verdict remains unchanged. This is a
user-pasted audit; the development computer does not possess that native file.

Verified source publishes both integrated IMU predictions (approximately 200 Hz)
and scan-corrected states (approximately 10 Hz) through the same odometry and TF
output. This allows an older scan correction after a newer prediction. Source
callsite IDs were absent in the old records, so individual old messages cannot
be labelled with independently recorded callsite provenance.

## Minimal implementation and preservation

New independent stage: `tools/hardware/mid360_lio_phased/`.
Original producer source and `install_mid360` remain byte-identical to their
399-entry manifest. Existing real raw captures remain usable without recapture.
The build copies that verified source into `build_mid360_lio_phased/src/`, applies
an output-only patch and installs only SPARK into `install_mid360_lio_phased`.
No deletes, resets or overwrites of old experiment evidence occurred.

The new producer contract is `scan_corrected_primary_v1`:

| Channel | Content | Nominal rate |
| --- | --- | --- |
| `/hardware/mid360/lio/odometry` | Scan-corrected sensor pose | 10 Hz |
| `/hardware/mid360/lio/odometry_predicted` | IMU-predicted sensor pose | 200 Hz |
| `/hardware/mid360/lio/odometry_mixed_legacy` | Every original output, including regressions | 210 Hz |
| TF `mid360_local_odom` → `mid360_imu` | Scan-corrected pose only | 10 Hz |

Output messages are local to each invocation, avoiding shared output-message
overwrites. Original timestamps and contents are retained; no sorting, dropping,
retimestamping, interpolation or monotonic gate relaxation occurs. Filter maths,
state update, preintegration, extrinsics, noise, scan rate, planar mode and other
estimation parameters remain unchanged. Existing Twist is still unfilled by the
producer, so zero Twist is not independent evidence of physical stationarity.

The patch implements output-phase separation. It does not guarantee navigation
timing, physical accuracy or all future per-phase monotonicity. Those are still
measured and gated in the new physical recording.

## Verification

1. Actual compiled production output API test: six original messages with two
   intentional regressions yield mixed=6, corrected=2, predicted=4 and TF=2.
   Complete Odometry equality checks prove channel identity; timestamps and poses
   are unmodified. Corrected and predicted streams increase separately.
2. Nine audit tests pass, including missing/mismatched records, broken CDR,
   missing channels, non-finite decoded pose, receipt stalls and TF mismatches.
3. Fifteen-second native recorder test in separate localhost-only domain 218,
   using an explicitly synthetic Python source: corrected=151, predicted=3020,
   mixed=3170, TF=151. Primary/prediction/TF regressions=0; mixed regressions=151.
   All 6492 native messages were written, without overflow/error. In the common
   device-time interval, corrected + predicted identities exactly equal mixed
   identities, and TF exactly matches corrected poses. Capture boundaries remain
   recorded and are explicitly counted outside this identity comparison.
   Independent re-audit matches the capture audit exactly. This test validates
   recording/audit plumbing and does not claim physical estimation validation.
4. Actual launch in isolated domain 219, with no sensors: parameter service
   confirms the new contract and original corrected configuration. `/proc` maps
   prove the loaded component is from the new install prefix, not the old library.
5. Original raw build still verifies all 399 entries. New local candidate seals
   148 entries including copied sources, output patcher, runtime tools and install
   products. This is not a complete OS or system-library image.

Evidence: `docs/evidence/mid360_lio_phased_20261017/`.
Local native fixture capture:
`/home/rmnav/AeroMind_mid360_lio_checks/static_phased_20261017_223423/`.

## Mini-PC continuation

Keep the real driver/IMU adapter running. Update and compile once:

```bash
cd ~/AeroMind_20261016
git pull --ff-only
BUILD_JOBS=2 bash tools/hardware/mid360_lio_phased/build.sh
```

Only after build success, stop the old **LIO** terminal with Ctrl+C. Keep the
separate real driver terminal running. In the stopped LIO terminal:

```bash
cd ~/AeroMind_20261016
bash tools/hardware/mid360_lio_phased/run.sh \
  /home/rm/AeroMind_mid360_captures/20261017_214624/summary.json
```

It rejects a duplicate primary LIO publisher and verifies original raw hashes,
fresh raw inputs, new build content and selected install prefix. After successful
initialization, keep radar/gimbal/chassis stationary. In another terminal:

```bash
cd ~/AeroMind_20261016
bash tools/hardware/mid360_lio_phased/observe_static.sh --seconds 30
```

The observer keeps every received serialized native message from all three
odometry channels and TF. An independent reader decodes CDR, recomputes all
critical finite values, per-phase timestamp order and unchanged 0.2 s continuity
limits, writer counts, message identity coverage, TF equality, source identity,
actual parameters and input/data hashes. It reports corrected sensor-pose drift
without inventing a physical accuracy threshold.

Physical cutover and new 30-second result remain pending user execution. Do not
use this change as an authorization for motor commands or navigation goals.
