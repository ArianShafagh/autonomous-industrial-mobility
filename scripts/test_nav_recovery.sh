#!/usr/bin/env bash
# System test of navigation failure handling in Gazebo (headless), with real obstacles.
#
#   test 1  box on section C's pickup pose, plan PICKUP:C;DELIVER
#           expect: C abandoned after the recovery ladder, robot returns to the charger,
#                   DELIVER still runs, mission ends plan_finished
#   test 2  box put on the charger after the robot has left, plan PICKUP:A;CHARGE:100
#           expect: charger unreachable -> robot halts, mission ends navigation_stuck
#
#   ./scripts/test_nav_recovery.sh [1|2|all]  results in logs/nav_recovery_<time>.txt
WS="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$WS/logs/nav_recovery_$(date +%Y%m%d_%H%M%S).txt"

latest_launch() { ls -t "$WS"/logs/*_launch.log | head -1; }

run_case() {   # name plan trigger_regex obstacle_poi
  local name="$1" plan="$2" trigger="$3" poi="$4"
  local before; before=$(latest_launch)
  bash "$WS/scripts/run_mission.sh" "$plan" balanced 1500 > "$WS/logs/.case.txt" 2>&1 &
  local runner=$!
  local L
  until L=$(latest_launch) && [ "$L" != "$before" ] && grep -qE "$trigger" "$L" 2>/dev/null; do sleep 2; done
  sleep 5
  bash "$WS/scripts/obstacle.sh" add "block_$poi" "$poi" > /dev/null 2>&1
  wait $runner
  {
    echo "=== $name  plan: $plan  obstacle on: $poi"
    cat "$WS/logs/.case.txt"
    grep -E "mission_executor\]: (\[[0-9]+\] .*(done|FAILED)|drive to|reached|NAVIGATION|MISSION)" "$L" \
      | sed 's/.*\[mission_executor\]: //' | cut -c1-220
    S=$(ls -t "$WS"/logs/*_mission_summary.yaml | head -1)
    sed -n '/^ended_by/p;/^navigation:/,/^prediction/p' "$S" | grep -v prediction
    echo "recovery column:"; cut -d, -f2,3,4,21- "$(ls -t "$WS"/logs/*_mission.csv | head -1)"
    echo
  } >> "$OUT"
}

WHICH="${1:-all}"
[ "$WHICH" = "all" ] || [ "$WHICH" = "1" ] && \
  run_case "test 1: unreachable section" "PICKUP:C;DELIVER" "Serving entity system service" C
[ "$WHICH" = "all" ] || [ "$WHICH" = "2" ] && \
  run_case "test 2: unreachable charger" "PICKUP:A;CHARGE:100" "\[1\] PICKUP:A from" charger
cat "$OUT"
