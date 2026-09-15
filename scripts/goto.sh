#!/usr/bin/env bash
# Send the robot to a named point of interest (or explicit coordinates) using Nav2.
#
#   ./scripts/goto.sh A | B | C | delivery | charger
#   ./scripts/goto.sh 1.5 -2.0 [yaw_rad]
#
# POI poses come from robofetch_factory/config/poi.yaml (generated from layout.yaml), so map
# coordinates are world coordinates. Needs navigation running and the initial pose set
# (./scripts/set_pose.sh).
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

if [ -z "${1:-}" ]; then
  echo "usage: $0 {$(ros2 run robofetch_factory poi | cut -d' ' -f1 | paste -sd'|')} | <x> <y> [yaw]"
  exit 1
fi

if [[ "$1" =~ ^-?[0-9.]+$ ]]; then
  X="$1"; Y="${2:?need a y coordinate}"; YAW="${3:-0.0}"; NAME="($X, $Y)"
else
  read -r X Y YAW < <(ros2 run robofetch_factory poi "$1") || exit 1
  NAME="$1"
fi

# yaw -> quaternion (rotation about z only)
read -r QZ QW < <(python3 -c "import math; print(math.sin($YAW/2), math.cos($YAW/2))")

echo "[goto] navigating to $NAME  x=$X y=$Y yaw=$YAW ..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}}}"
