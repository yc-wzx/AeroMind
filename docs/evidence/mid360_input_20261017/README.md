# MID360 input tool verification (2026-10-17)

Hardware was NOT attached to this development machine. No new real-world LIO,
body pose, motor movement or navigation qualification is claimed.

Final result is verification.json; final_build_v2.txt contains the successful
build, actual production C++ MID360 preprocessing probe and 16 tests. The separate
imu_ros_interface_fixture.json is a synthetic ROS domain214 test. Gate/CLI tests
confirm absent raw data and absent evidence cannot pass or start LIO. No real
navigation goal or velocity was published.

Earlier final_build.txt retains a test-fixture failure: comparison reserialized
new messages instead of comparing the captured serialized bytes (CDR padding is
not an invariant across serializations). The corrected test compares exact bytes
passed to the writer against bag readback. A prior development fixture also needed
array-to-bytes conversion for row-padding input. Production sensor/LIO math was
not altered for these fixture repairs. build.txt is the initial package build.

missing_input_expected_rejection.json is a deliberately empty-input negative
fixture, NOT a real hardware failure or a passing capture. Historical raw results
and simulation baselines are retained elsewhere and not overwritten.
