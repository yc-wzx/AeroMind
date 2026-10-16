# 2026-10-16 deployment evidence

Historical original-machine complete-flow PASS (260.74 simulated seconds) and
LIO optimization summaries are reference evidence only; raw original captures
remain in the original checkout and are not shipped as a new qualification.

New clean build: clean_build_receipt.json, final_build_recheck.txt and
rebuilt_*_tests.txt. New startup-only observation: startup_verification.json,
startup_actual_runtime_inputs.json, startup_launch.txt and the losslessly gzipped
startup_observations.json.gz. No goal was sent; no new full route or hardware test.
Shutdown SIGINT tracebacks are preserved in the launch log.

local_rebuild_manifest.json contains this machine's build paths and hashes, not
hashes required on another computer. The destination build writes its own.
provenance.json and portable_equivalence.json identify original copied modules and
path-only changes; preservation_verification.json confirms original inputs remain.
original_working_diff.patch/status preserve the pre-existing original dirty work.

The manifest lists selected local sources/installed products, not a full system
image. These receipts do not establish real MID-360, motor/serial or official
arena readiness. TRAINING-ONLY / PROVISIONAL; COMPETITION ARENA NOT VERIFIED;
CONTACT DATA UNAVAILABLE.
