#!/bin/bash
# capture_figure.sh - RViz2 -> publication figure, on THIS laptop.
#
#   ./capture_figure.sh <out.png> [rviz_config] [--settle N]
#
# WHAT ACTUALLY WORKS HERE (verified 2026-08-28, not assumed)
# -----------------------------------------------------------
#  * RViz2 MUST run with a real display. This laptop is GNOME on Wayland with
#    Xwayland at :0 (Intel UHD 620, direct rendering yes).
#  * RViz2 will NOT start under the default Qt platform plugin. It dies with
#      RenderingAPIException: Invalid parentWindowHandle (wrong server or screen)
#    Setting QT_QPA_PLATFORM=xcb fixes it completely (0 render errors after).
#    This script exports it; do the same in any shell you launch RViz from.
#  * HEADLESS IS NOT AVAILABLE HERE. Xvfb is not installed, and installing it
#    is out of scope. So figure export needs the laptop's own session; you
#    cannot do it over a plain SSH pipe with no display.
#  * Capture: ImageMagick `import -window <id>` works and is what this uses.
#    xdotool and wmctrl are NOT installed, so the window is located with
#    `xwininfo -root -tree` instead.
#
# RESOLUTION. `import` captures the window at its on-screen size, so the
# capture is 1920x937 on this display - NOT arbitrarily high resolution.
# There is no supersampling: RViz2 has no offscreen render-at-N-times mode,
# and no --resolution flag. To get a larger figure you must give RViz a larger
# window before capturing. Two honest options:
#   1. Resize the RViz window (or the whole output) larger, then capture.
#   2. Capture at 1920x937 and upscale, which does NOT add detail - say so
#      rather than pretending it is a high-res render.
# For a two-column paper figure 1920 px wide at ~300 dpi is about 6.4 in, which
# is usually enough. Crop the 3-D viewport out of the chrome with --crop.
set -uo pipefail

OUT="${1:?usage: ./capture_figure.sh <out.png> [rviz_config] [--settle N] [--crop]}"
shift
CFG=""; SETTLE=6; CROP=0
while [ $# -gt 0 ]; do case "$1" in
  --settle) SETTLE="$2"; shift 2 ;;
  --crop) CROP=1; shift ;;
  *) CFG="$1"; shift ;;
esac; done

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb          # REQUIRED - see header

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u; source /opt/ros/humble/setup.bash 2>/dev/null; set -u

STARTED=0
if ! pgrep -f '[r]viz2' >/dev/null; then
  [ -z "$CFG" ] && CFG="$HERE/viz/fleet_overview.rviz"
  echo "starting rviz2 with $CFG ..."
  setsid nohup rviz2 -d "$CFG" --ros-args -p use_sim_time:=true \
      >/tmp/capture_rviz.log 2>&1 </dev/null &
  disown; STARTED=1
  for _ in $(seq 30); do
    xwininfo -root -tree 2>/dev/null | grep -q "RViz" && break; sleep 1; done
fi

echo "letting the scene settle for ${SETTLE}s ..."
sleep "$SETTLE"

WIN=$(xwininfo -root -tree 2>/dev/null | grep -oP '0x[0-9a-f]+(?=\s+"[^"]*RViz")' | head -1)
if [ -z "$WIN" ]; then
  echo "ERROR: no RViz window found on $DISPLAY."
  echo "  If you are on a headless/SSH session this cannot work: Xvfb is not"
  echo "  installed. Run this from the laptop's own desktop session."
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
import -window "$WIN" "$OUT"
if [ "$CROP" = 1 ]; then
  # drop the left dock (Displays/Views) and the menu strip, keeping the 3-D
  # viewport only. Geometry matches the default 1920-wide layout.
  convert "$OUT" -crop '1358x854+562+45' +repage "$OUT"
fi
identify "$OUT"
[ "$STARTED" = 1 ] && echo "(rviz2 was started by this script and is still running)"
