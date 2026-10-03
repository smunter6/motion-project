#!/usr/bin/env bash
# The same scripted session as `demo_session.sh`, but with the viz window.
#
# The window needs a display server (see app/CLAUDE.md for the WSL setup) and
# about 2 seconds to start before the first command can land, so the session
# opens with a sleep. The window closes when the control loop exits, which
# happens on `quit` or stdin EOF, so the session ends with a pause to leave the
# plots on screen. Close the window early to stop the app.
#
# Run from the workspace root.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

cargo build -p app

LINGER_SECONDS=${1:-15}   # how long the window stays open after the last move

{
    sleep 2              # let the viz window spin up

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
    echo "move axisGroup1 150 50"
    sleep 3
    echo "status"
    sleep 1
    echo "movepath axisGroup1 2 160 40 120 -80"
    sleep 6
    echo "status"

    sleep "$LINGER_SECONDS"
    echo "quit"

} | ./target/debug/app
