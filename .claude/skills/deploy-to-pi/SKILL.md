---
name: deploy-to-pi
description: Cross-compile this workspace from WSL for a Raspberry Pi target and deploy it. Use when building for, deploying to, or running on the Pi, or when EtherCAT bus testing on real hardware comes up.
---

# Deploying to the Raspberry Pi

Development and host-side unit tests happen in WSL. Anything touching a real
EtherCAT bus runs on the Pi — WSL2's virtualized NAT networking can't do raw L2
EtherCAT reliably.

## Target

`aarch64-unknown-linux-gnu` (64-bit Pi OS).

## Cross-compiling

Use either `cross` or `cargo-zigbuild` — the workspace has no target-specific
build script, so a plain cross-compile is enough:

```
cargo zigbuild --release --target aarch64-unknown-linux-gnu
```

Don't build `app`'s viz for the Pi. Viz is a host-only dev tool (eframe /
egui_plot, needs a display); the Pi runs the control loop headless.

## Deploy loop

build → `rsync` the binary to the Pi → `ssh` in and run it.

## Notes

- `motion-core` is dependency-free and cross-compiles unchanged.
- Real drive testing needs an EtherCAT backend (EtherCRAB + CiA 402), which this
  repository does not include.
