#!/usr/bin/env bash
# Cold start of the autonomous factory robot: build, source, launch (Gazebo + RViz).
#
#   ./scripts/run.sh                              demo plan, balanced scenario, with GUI
#   ./scripts/run.sh --headless                   no Gazebo GUI, no RViz
#   ./scripts/run.sh --no-build                   skip colcon build
#   ./scripts/run.sh scenario:=high_demand        any mission.launch.py argument is passed on
#   ./scripts/run.sh plan:="PICKUP:B;DELIVER;CHARGE:90"
#
# No operator input is needed after start: the robot localises on the charger and executes the
# plan by itself. Until the AI decision service exists (WP7) the robot follows a scripted plan.
# For unattended runs that stop by themselves use ./scripts/run_mission.sh instead.
#
# Stopping is MANUAL: this script never kills a running simulation, it refuses to start next to
# one. Press Ctrl+C and run ./scripts/stop.sh when you want the simulation gone.
# NOTE: no `set -u`; ROS setup.bash references unbound variables and would abort.
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"

DEMO_PLAN="PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100"
BUILD=1
HAS_PLAN=0
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --headless) LAUNCH_ARGS+=('headless:=true') ;;
    --no-build) BUILD=0 ;;
    plan:=*)    HAS_PLAN=1; LAUNCH_ARGS+=("$arg") ;;
    *)          LAUNCH_ARGS+=("$arg") ;;
  esac
done
[ "$HAS_PLAN" -eq 1 ] || LAUNCH_ARGS+=("plan:=$DEMO_PLAN")

# Nothing is ever stopped automatically - a simulation or test already running is not killed
# behind your back. Stop it yourself with ./scripts/stop.sh.
if RUNNING=$(bash "$WS/scripts/stop.sh" --running); then
  echo "[run] $RUNNING simulation process(es) are already running."
  echo "[run] Two simulations publish two /clock streams and BOTH runs break."
  echo "[run] Stop the old one first:   ./scripts/stop.sh"
  exit 1
fi

echo "[run] 1/3  sourcing ROS 2 Jazzy ..."
source /opt/ros/jazzy/setup.bash

if [ "$BUILD" -eq 1 ]; then
  echo "[run] 2/3  building the workspace ..."
  cd "$WS"
  source "$WS/venv/bin/activate"
  colcon build --symlink-install
else
  echo "[run] 2/3  skipping build (--no-build)"
fi

echo "[run] 3/3  sourcing the workspace and launching ..."
source "$WS/install/setup.bash"

echo
echo "  Gazebo and RViz will open. The robot starts on the blue charger (south-west)."
echo "  Sections: A red (north-west), B yellow (north-east), C purple (south-east);"
echo "  delivery point green (south-centre). Expect ~30 s before the robot first moves."
echo "  Results: logs/<run_id>_mission.csv and _mission_summary.yaml"
echo "  Press Ctrl+C to stop, then run ./scripts/stop.sh before launching again."
echo
exec ros2 launch robofetch_bringup mission.launch.py "${LAUNCH_ARGS[@]}"
