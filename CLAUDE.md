# CLAUDE.md

Working instructions for Claude Code. What the project is, how to run it, and
what is and isn't implemented is in [`README.md`](README.md).

## Constraints to preserve

- `motion-core` does no I/O, has no dependencies, and does not allocate in the
  per-cycle path. Nothing touching a NIC, a drive, or a window goes in it.
- The planner reaches a backend only through the `AxisGroup` trait in
  `axis-backend`. It never learns whether a simulation or real drives are behind
  it.
- Trajectories are pure functions of elapsed time. Only the plant (`backend-sim`,
  or a real servo) steps by `dt`.
- Every profile is seeded from commanded state, not feedback.
- `app` business logic gates on `AxisState` only. `Ds402State` is for printing
  transitions.

## Working in this repo

- Never use `sed`, `awk`, or `cat` to read or edit files. Use the Read, Edit,
  and Write tools.
- Verify interactive changes against the built binary (`./target/debug/app`),
  not `cargo run`; piped stdin races `cargo run`'s startup.
- Scripted or automated sessions run `--headless`. The exception is a change
  that is about the visualization itself.
- Develop and test in WSL on the Linux filesystem. Real EtherCAT bus testing
  happens on the Pi. See the `deploy-to-pi` skill.
- `app/CLAUDE.md` covers getting the viz window to open under WSL.
