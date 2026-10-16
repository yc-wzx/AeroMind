# Portable rule900 training simulation

See [deployment instructions](../../docs/PROVINCIAL_RULE900_DEPLOYMENT.md).
Private copies preserve the actual tested Python dependency closure; runtime
modules resolve from this package rather than old overlay workspaces.
Plugins and LIO are rebuilt locally. No real driver or electrical serial opens.
The LIO wrapper requires explicit `--simulation-only true` and validates the local
build manifest. Source paths are discovered relative to the checkout.

This package preserves the conservative 0.30 m circular full-robot radius,
fixed physical chassis, inverted sensor on yaw-only gimbal, and existing guards.
TRAINING-ONLY / PROVISIONAL; COMPETITION ARENA NOT VERIFIED.
