#!/usr/bin/env bash
# Example of driving `app` non-interactively: pipe a scripted sequence of
# commands into its stdin instead of typing them by hand.
#
# Runs `--headless` (no viz window), which is the default for any scripted
# or automated session — see `app`'s crate docs. That also means no startup
# sleep is needed: the control loop is running before the first command is
# drained. Drop the flag to watch the same sequence in the viz window, and
# add a `sleep 2` at the top if you do, to let the window spin up.
#
# `app` reads one command per line and answers immediately (see
# app/src/main.rs's `read_commands`), but some commands take real control
# cycles to land — enabling an axis is a 3-cycle DS402 sequence at 250 Hz
# (~12 ms) — so a following command that depends on it (e.g. `move` right
# after `enable`) still needs a short sleep first. Run from the workspace
# root.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

cargo build -p app

{
    echo "enable axisGroup0"
    echo "enable axisGroup1"
    sleep 0.2            # wait for the enable sequences to reach OperationEnabled

    # A Cartesian group: the axes are X and Y, so kinematics is a
    # pass-through.
    echo "move axisGroup0 100 100"
    sleep 4
    echo "movepath axisGroup0 file waypoints.txt"

    # The SCARA arm: the same Cartesian commands, but axis2/axis3 are rotary
    # joints in radians and every setpoint goes through inverse kinematics.
    # `status` shows both — joint values per axis, TCP position per group.
    echo "move axisGroup1 150 50"
    sleep 3
    echo "status"
    sleep 1
    echo "movepath axisGroup1 2 160 40 120 -80"
    sleep 6
    echo "status"

    echo "quit"

} | ./target/debug/app --headless
