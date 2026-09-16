#!/usr/bin/env bash
# Put a box obstacle into the running Gazebo world, or remove it. Used to test how the robot
# handles blocked routes and unreachable goals.
#
#   ./scripts/obstacle.sh add <name> <x> <y> [size_x] [size_y]     (or a POI name instead of x y)
#   ./scripts/obstacle.sh remove <name>
#
#   ./scripts/obstacle.sh add block_C C          # 0.6 x 0.6 m box right on section C's pickup pose
WS="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
WORLD=factory_maze

case "${1:-}" in
  add)
    NAME="${2:?name}"
    if [[ "${3:-}" =~ ^-?[0-9.]+$ ]]; then X="$3"; Y="${4:?y}"; shift 4
    else read -r X Y _ < <(ros2 run robofetch_factory poi "${3:?x or POI}") || exit 1; shift 3; fi
    SX="${1:-0.6}"; SY="${2:-0.6}"; H=0.5
    SDF="<sdf version='1.10'><model name='$NAME'><static>true</static><link name='l'>\
<collision name='c'><geometry><box><size>$SX $SY $H</size></box></geometry></collision>\
<visual name='v'><geometry><box><size>$SX $SY $H</size></box></geometry>\
<material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>\
</link></model></sdf>"
    ros2 run ros_gz_sim create -world "$WORLD" -name "$NAME" -x "$X" -y "$Y" -z $(python3 -c "print($H/2)") -string "$SDF"
    ;;
  remove)
    gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
      --timeout 3000 --req "name: '${2:?name}' type: MODEL"
    ;;
  *) echo "usage: $0 add <name> <x> <y>|<poi> [sx] [sy]  |  $0 remove <name>"; exit 1 ;;
esac
