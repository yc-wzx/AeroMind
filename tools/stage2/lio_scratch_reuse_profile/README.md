# Source-only simulation LIO profile

This is the qualified original source candidate, including scratch-array reuse.
The deployment build compiles it on the destination computer and runs the actual
production prediction, calibration and keyframe C++ tests. It generates a new
machine-local build manifest, not a replacement historical qualification.
No runtime binaries, raw captures, or old binary-hash approvals are copied here.
Historical provenance fields inside source/production_manifest.json are metadata,
not an absolute runtime path requirement.

The existing readonly_source.sh mount wrapper remains in use and must succeed.
Unbiased IMU / instantaneous cloud flags are simulation-only assumptions.
Do not use this configuration as a real MID-360/IMU profile.
