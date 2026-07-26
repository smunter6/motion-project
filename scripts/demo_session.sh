#!/usr/bin/env bash
# Example of driving `app` non-interactively: pipe a scripted sequence of
# commands into its stdin instead of typing them by hand.
#
# `app` reads one command per line and answers immediately (see
# app/src/main.rs's `read_commands`), but some commands take real control
# cycles to land — enabling an axis is a 3-cycle DS402 sequence at 250 Hz
# (~12 ms) — so a following command that depends on it (e.g. `move` right
# after `enable`) needs a short sleep first. Run from the workspace root.
#
# Note: `app` also opens a viz window (eframe/glow) on the main thread and
# won't exit until it closes; `quit` below closes it for you.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

cargo build -p app

{
    sleep 2              # let the control loop + viz window spin up
    echo "enable axisGroup0"
    sleep 0.2            # wait for the enable sequence to reach OperationEnabled
    echo "move axisGroup0 100 100"
    sleep 2              # let the move actually run
    echo "status"
    sleep 2
    echo "movepath axisGroup0 file waypoints.txt"
    sleep 60

} | cargo run -p app
