# Static LIO configuration correction

Final proof: probe_test.txt (actual class/output method, synthetic isolated ROS215),
probe_build_v4.txt, python_tests.txt, raw_manifest_preserved.json and negative
missing_lio_expected_rejection.txt. Earlier probe_build[v2/v3] retain test-harness
Eigen/IKFoM/ikd include setup failures; production source/algorithm was not changed.
The harness uses compiler access-control relaxation only to call the real private
output APIs; this is not a production flag or system setting.

Original raw sensor inputs, SDK, binaries and manifest are untouched. Only the
new sibling LIO stage supplies corrected visualization mode imu. Old raw capture
remains reusable and unchanged. user_reported_raw_capture.json summarizes the
user's pasted results, not an independent replay of remote bag files. Raw/SI
count difference awaits the new exact receipt-window report. No packet-loss,
real LIO drift, chassis calibration, navigation or motor-control qualification.
