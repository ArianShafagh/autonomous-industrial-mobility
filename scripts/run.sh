#!/usr/bin/env bash
# Start the autonomous factory robot.
#
#   ./scripts/run.sh                         INTERACTIVE: asks for scenario, AI model, view and
#                                            shift length, then starts everything
#   ./scripts/run.sh --list                  show the scenarios and models, start nothing
#
# Non-interactive (for scripts and repeat runs) - any argument skips the questions:
#   ./scripts/run.sh scenario:=heavy_load model:=ns shift_s:=600.0
#   ./scripts/run.sh scenario:=low_battery_start model:=ppo --headless
#   ./scripts/run.sh model:=fallback                        no AI: the robot's built-in rules
#   ./scripts/run.sh model:= plan:="PICKUP:B;DELIVER"      no decisions: a scripted sequence
#   ./scripts/run.sh ... --no-build                         skip colcon build
#
# After the start there is no operator input at all: the robot localises on the charger, asks the
# chosen model what to do next, and works the shift by itself. Watch it on http://localhost:8000.
#
# Stopping is MANUAL: this script never kills a running simulation, it refuses to start next to
# one. Press Ctrl+C and run ./scripts/stop.sh when you want the simulation gone.
# NOTE: no `set -u`; ROS setup.bash references unbound variables and would abort.
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
SCENARIO_DIR="$WS/src/robofetch_factory/config/scenarios"

# Only used when the AI is switched off with model:= - otherwise the model decides everything.
DEMO_PLAN="PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100"

MODELS=(ns ns_symbolic_only ppo rule fallback)
MODEL_TEXT=(
  "neuro-symbolic      rules + neural scorer (the thesis model)"
  "symbolic only       the rules alone, no neural network"
  "PPO                 deep reinforcement learning (comparison)"
  "rule-based          simple reference policy"
  "no AI               built-in fallback rules only (AI service off)"
)

scenario_names() { for f in "$SCENARIO_DIR"/*.yaml; do basename "$f" .yaml; done; }
scenario_description() { grep -m1 '^description:' "$SCENARIO_DIR/$1.yaml" | sed 's/^description: *//'; }

list_everything() {
  echo "Scenarios (src/robofetch_factory/config/scenarios/):"
  local i=1
  for s in $(scenario_names); do
    printf "  %2d) %-18s %s\n" "$i" "$s" "$(scenario_description "$s" | cut -c1-90)"
    i=$((i + 1))
  done
  echo
  echo "Decision models:"
  for i in "${!MODELS[@]}"; do printf "  %2d) %s\n" "$((i + 1))" "${MODEL_TEXT[$i]}"; done
}

ask() {   # ask "question" default  -> echo answer
  local answer
  read -r -p "$1 [$2]: " answer
  echo "${answer:-$2}"
}

# ---------------------------------------------------------------------------------- arguments
BUILD=1
HAS_PLAN=0
NO_MODEL=0
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --list)      list_everything; exit 0 ;;
    --headless)  LAUNCH_ARGS+=('headless:=true') ;;
    --no-build)  BUILD=0 ;;
    plan:=*)     HAS_PLAN=1; LAUNCH_ARGS+=("$arg") ;;
    model:=)     NO_MODEL=1; LAUNCH_ARGS+=('model:=' 'ai:=false') ;;
    model:=fallback) LAUNCH_ARGS+=('model:=fallback' 'ai:=false') ;;
    *)           LAUNCH_ARGS+=("$arg") ;;
  esac
done

# ------------------------------------------------------------------- interactive (no arguments)
if [ "$#" -eq 0 ]; then
  if [ ! -t 0 ]; then
    echo "[run] no arguments and no terminal to ask in - see ./scripts/run.sh --list"; exit 1
  fi
  mapfile -t SCENARIOS < <(scenario_names)
  echo
  echo "================ Autonomous factory robot ================"
  echo
  list_everything
  echo

  default_scenario=1
  for i in "${!SCENARIOS[@]}"; do [ "${SCENARIOS[$i]}" = "balanced" ] && default_scenario=$((i + 1)); done
  while :; do
    pick=$(ask "Scenario number" "$default_scenario")
    if [[ "$pick" =~ ^[0-9]+$ ]] && [ "$pick" -ge 1 ] && [ "$pick" -le "${#SCENARIOS[@]}" ]; then
      SCENARIO="${SCENARIOS[$((pick - 1))]}"; break
    fi
    echo "  please enter a number between 1 and ${#SCENARIOS[@]}"
  done

  while :; do
    pick=$(ask "Decision model number" "1")
    if [[ "$pick" =~ ^[0-9]+$ ]] && [ "$pick" -ge 1 ] && [ "$pick" -le "${#MODELS[@]}" ]; then
      MODEL="${MODELS[$((pick - 1))]}"; break
    fi
    echo "  please enter a number between 1 and ${#MODELS[@]}"
  done

  echo
  echo "View:  1) Gazebo + RViz windows     2) headless (faster; watch on the dashboard only)"
  while :; do
    pick=$(ask "View" "1")
    case "$pick" in 1) HEADLESS=false; break ;; 2) HEADLESS=true; break ;;
                    *) echo "  please enter 1 or 2" ;; esac
  done

  echo
  echo "Shift length in minutes of simulated time (the full shift in params.yaml is 60)."
  while :; do
    minutes=$(ask "Minutes" "10")
    if [[ "$minutes" =~ ^[0-9]+$ ]] && [ "$minutes" -ge 1 ]; then break; fi
    echo "  please enter a whole number of minutes"
  done

  LAUNCH_ARGS=("scenario:=$SCENARIO" "headless:=$HEADLESS" "shift_s:=$((minutes * 60)).0"
               "model:=$MODEL")
  # The fallback rules run inside the robot's executor; the AI service is not needed for them.
  [ "$MODEL" = "fallback" ] && LAUNCH_ARGS+=('ai:=false')

  echo
  echo "  scenario : $SCENARIO - $(scenario_description "$SCENARIO" | cut -c1-80)"
  echo "  model    : $MODEL"
  echo "  view     : $([ "$HEADLESS" = true ] && echo headless || echo 'Gazebo + RViz')"
  echo "  shift    : $minutes min"
  confirm=$(ask "Start? (y/n)" "y")
  [[ "$confirm" =~ ^[Yy] ]] || { echo "[run] cancelled"; exit 0; }
fi

# model:= (empty) means "no decisions at all, follow a script"; without an explicit plan the
# demo sequence is used so the robot still does something visible.
if [ "$NO_MODEL" -eq 1 ] && [ "$HAS_PLAN" -eq 0 ]; then
  LAUNCH_ARGS+=("plan:=$DEMO_PLAN")
fi

# Nothing is ever stopped automatically - a simulation or test already running is not killed
# behind your back. Stop it yourself with ./scripts/stop.sh.
if RUNNING=$(bash "$WS/scripts/stop.sh" --running); then
  echo "[run] $RUNNING simulation process(es) are already running."
  echo "[run] Two simulations publish two /clock streams and BOTH runs break."
  echo "[run] Stop the old one first:   ./scripts/stop.sh"
  exit 1
fi

echo
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
echo "  The robot starts on the blue charger (south-west) and begins by itself after ~30 s."
echo "  Sections: A red (north-west), B yellow (north-east), C purple (south-east);"
echo "  delivery point green (south-centre)."
echo "  Watch it:  http://localhost:8000   (read-only dashboard)"
echo "  Results:   logs/<run_id>_mission_summary.yaml when the shift ends"
echo "  Stop:      Ctrl+C here, then ./scripts/stop.sh"
echo
exec ros2 launch robofetch_bringup mission.launch.py "${LAUNCH_ARGS[@]}"
