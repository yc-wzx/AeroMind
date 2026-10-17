# Real MID-360 sensor-local LIO startup Context correction — 2026-10-17

The user's real raw-input capture passed. The following stationary observer
record contained zero messages, no odometry publishers and no parameter service:
`/home/rm/AeroMind_mid360_lio_checks/static_20261017_220539/summary.json`.
That record remains INCOMPLETE, not a physical drift or navigation result.

The reported startup traceback was independently reproduced by calling the
unmodified production `live_ready()` in local ROS 2 Humble with its default
Context uninitialized: `AttributeError: __enter__`. The function creates a scoped
Context, while `rclpy.spin_once(node)` selects the global executor bound to the
default Context. LIO was never launched in that attempt.

## Minimal correction

Only `tools/hardware/mid360_lio/start.py` runtime logic changes: create an explicit
SingleThreadedExecutor with the same scoped Context, attach the actual node,
spin that executor, and shut it down before destroying the scoped Context.
The default live readiness deadline remains eight seconds. The optional function
argument permits short negative tests; the production CLI uses the same default.
No driver, timestamps, LIO estimation parameters, raw capture tools, source/install
SDK inputs, navigation or motor command logic changes.

## Evidence

The five new tests invoke production `live_ready()` using real ROS/DDS callbacks,
not a copied expression. They require separate localhost-only ROS domain 217:

- valid synthetic PointCloud2/IMU sources pass twice while the default Context is
  uninitialized and access to the global executor is prohibited;
- missing inputs fail cleanly;
- a duplicate odometry publisher remains rejected;
- non-finite IMU input remains rejected;
- a wrong sensor publisher remains rejected.

The four existing stationary-tool tests also pass. Their existing writer-count
test emits a ResourceWarning for its line-count read handle; it does not fail and
is outside this startup correction. Raw preflight still verifies the exact 399
local source/install/SDK input entries. This is local evidence, not an independent
replay of the user's mini-PC raw bag. The original raw files were not accessed or
changed by this fix.

See `docs/evidence/mid360_lio_context_20261017/` for the before-fix source, actual
traceback, both test logs, manifest check, patch, verification report and hashes.

## Mini-PC continuation

Keep the sensors-only real driver running. Run `git pull --ff-only`; no C++
recompilation or raw recapture is required. Then retry:

```bash
bash tools/hardware/mid360_lio/run.sh /home/rm/AeroMind_mid360_captures/20261017_214624/summary.json
```

Only after LIO stays running, keep radar/gimbal/chassis stationary and collect in
another terminal:

```bash
bash tools/hardware/mid360_lio/observe_static.sh --seconds 30
```

Physical sensor-local LIO initialization, stationary drift and future body-frame
integration remain PENDING. No real motor command or navigation goal was sent.
