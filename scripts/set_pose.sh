#!/usr/bin/env bash
# Tell AMCL where the robot is, so it can localize (same as RViz "2D Pose Estimate").
#
#   ./scripts/set_pose.sh              -> the charger, where the robot spawns
#   ./scripts/set_pose.sh <poi>        -> a named point of interest
#   ./scripts/set_pose.sh x y [yaw]    -> arbitrary pose
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

if [[ "${1:-}" =~ ^-?[0-9.]+$ ]]; then
  X="$1"; Y="${2:?need a y coordinate}"; YAW="${3:-0.0}"
else
  read -r X Y YAW < <(ros2 run robofetch_factory poi "${1:-charger}") || exit 1
fi
read -r QZ QW < <(python3 -c "import math; print(math.sin($YAW/2), math.cos($YAW/2))")

echo "[set_pose] telling AMCL the robot is at x=$X y=$Y yaw=$YAW ..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}, covariance: [0.05,0,0,0,0,0, 0,0.05,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.03]}}"
