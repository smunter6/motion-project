#!/usr/bin/env bash
# Draw a converted SVG on `axisGroup0`.
#
# `svg_to_waypoints` emits a directory of `contourNN.txt` waypoint files plus
# a `session.txt` and a `length.txt`. `session.txt` is piped input for `app`,
# not an executable; this script feeds it to the app's stdin.
#
# Usage:  scripts/utilities/draw_svg.sh [--headless] <contour-dir> [vmax]
#
# `<contour-dir>` is the `--out-dir` given to `svg_to_waypoints`. Run from
# anywhere; the script changes to the workspace root, so the paths inside
# `session.txt` (relative to the workspace root) resolve.
#
# `--headless` is passed straight through to `app` (no viz window). The default
# is the window, matching `app`.
#
# `vmax` (default 12 mm/s) is appended to every path move. At the configured
# 50 mm/s, glyph corners have curvature around 0.5/mm, so the centripetal term
# v^2*k reaches ~1250 mm/s^2 against a 200 mm/s^2 limit. Nothing bounds that
# and `backend-sim` doesn't show it, so slowing the feedrate is the only
# control.
#
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir/../.."

opts=()
startup_sleep=2   # the viz window needs a moment before commands land
positional=()
for arg in "$@"; do
    case "$arg" in
        --headless)
            opts=(--headless)
            startup_sleep=0   # headless is running before the first command
            ;;
        -*)
            echo "unknown option $arg" >&2
            echo "usage: draw_svg.sh [--headless] <contour-dir> [vmax]" >&2
            exit 1
            ;;
        *) positional+=("$arg") ;;
    esac
done

if [[ ${#positional[@]} -lt 1 ]]; then
    echo "usage: draw_svg.sh [--headless] <contour-dir> [vmax]" >&2
    exit 1
fi
dir="${positional[0]}"
vmax="${positional[1]:-12}"
session="$dir/session.txt"

if [[ ! -f "$session" ]]; then
    echo "no $session — run svg_to_waypoints first" >&2
    exit 1
fi

cargo build -p app

# `app` treats stdin EOF as `quit` (see read_commands), so when this script's
# input ends the control loop exits and any still-queued contour is discarded.
# The script therefore waits for the drawing to finish before closing stdin.
#
# svg_to_waypoints writes the travel distance (contours plus rapids) to
# length.txt. Estimating from the waypoint count undershoots: a long straight
# run is two waypoints.
if [[ ! -f "$dir/length.txt" ]]; then
    echo "no $dir/length.txt — regenerate with svg_to_waypoints" >&2
    exit 1
fi
mm=$(< "$dir/length.txt")
# Add 50% for the accel/decel ramps and jerk-filter window on each move, then
# a 20 s floor. Integer arithmetic, so `bc` isn't required.
wait_s=$(( 3 * mm / (2 * vmax) + 20 ))
echo "drawing $dir: ${mm} mm at $vmax mm/s, waiting ${wait_s}s"

{
    sleep "$startup_sleep"
    echo "enable axisGroup0"
    sleep 0.2         # the enable is a 3-cycle DS402 sequence at 250 Hz

    # Append the feedrate to every move. `buffered` must stay the last token,
    # so the limit goes in ahead of it.
    while IFS= read -r cmd; do
        [[ -z "$cmd" ]] && continue
        echo "${cmd% buffered} $vmax buffered"
    done < "$session"

    sleep "$wait_s"
    echo "status"
    sleep 0.2
    echo "quit"
} | ./target/debug/app "${opts[@]}"
