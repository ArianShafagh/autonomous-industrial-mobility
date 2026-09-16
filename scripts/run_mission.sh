#!/usr/bin/env bash
# Run one complete headless mission and stop everything afterwards.
#
#   ./scripts/run_mission.sh "<plan>" [scenario] [timeout_s]
#   ./scripts/run_mission.sh "PICKUP:B;PICKUP:A;DELIVER;CHARGE:100" balanced 1200
#
# Exit code 0 = MISSION COMPLETE, 1 = aborted/failed, 2 = timed out.
# Launch output: logs/<timestamp>_launch.log; mission results: logs/<run_id>_mission*.
WS="$(cd "$(dirname "$0")/.." && pwd)"
PLAN="${1:?usage: $0 \"<plan>\" [scenario] [timeout_s]}"
SCENARIO="${2:-balanced}"
TIMEOUT="${3:-1800}"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
# Refuse to start on top of a running simulation (never kill someone else's run); the cleanup at
# the END below only stops the simulation this script started itself.
if RUNNING=$(bash "$WS/scripts/stop.sh" --running); then
  echo "[run_mission] refusing to start: $RUNNING simulation process(es) already running."
  echo "[run_mission] stop them first with ./scripts/stop.sh"
  exit 3
fi

mkdir -p "$WS/logs"
LOG="$WS/logs/$(date +%Y%m%d_%H%M%S)_launch.log"
ros2 launch robofetch_bringup mission.launch.py headless:=true scenario:="$SCENARIO" plan:="$PLAN" \
  > "$LOG" 2>&1 &

START=$(date +%s)
RESULT=2
while [ $(( $(date +%s) - START )) -lt "$TIMEOUT" ]; do
  if grep -q "MISSION COMPLETE" "$LOG"; then RESULT=0; break; fi
  if grep -q "mission aborted\|MISSION ENDED\|\[mission_executor-[0-9]*\]: process has died" "$LOG"; then RESULT=1; break; fi
  sleep 5
done
ELAPSED=$(( $(date +%s) - START ))

bash "$WS/scripts/stop.sh" >/dev/null
case $RESULT in
  0) echo "[run_mission] COMPLETE in ${ELAPSED}s wall  (log $LOG)";;
  1) echo "[run_mission] FAILED after ${ELAPSED}s wall  (log $LOG)";;
  2) echo "[run_mission] TIMEOUT after ${ELAPSED}s wall (log $LOG)";;
esac
exit $RESULT
