#!/usr/bin/env bash
# Start the autonomous factory robot from the starter config (config/run.yaml).
#
#   ./scripts/run.sh                         read config/run.yaml; with `ask: true` show the menu
#                                            (scenario, model, planner, view, minutes) with the
#                                            file's values as defaults, then start
#   ./scripts/run.sh --yes                   start straight from the file, no questions
#   ./scripts/run.sh --ask                   show the menu even when the file says ask: false
#   ./scripts/run.sh --config other.yaml     use another starter file
#   ./scripts/run.sh --list                  show scenarios, models and planners, start nothing
#   ./scripts/run.sh --dry-run               resolve everything, print the launch command, start nothing
#
# Command-line values override the file for this run only (and skip the menu, for repeat runs):
#   ./scripts/run.sh scenario:=heavy_load model:=ppo planner:=Smac2D shift_s:=600.0
#   ./scripts/run.sh model:=fallback --headless --no-build
#   ./scripts/run.sh model:= plan:="PICKUP:B;DELIVER"      no decisions: a scripted sequence
#   Any other name:=value is passed to mission.launch.py unchanged (e.g. rviz:=false).
#
# After the start there is no operator input at all: the robot localises on the charger, asks the
# chosen model what to do next, and drives with the chosen planner. Watch on http://localhost:8000.
#
# Stopping is MANUAL: this script never kills a running simulation, it refuses to start next to
# one. Press Ctrl+C and run ./scripts/stop.sh when you want the simulation gone.
# NOTE: no `set -u`; ROS setup.bash references unbound variables and would abort.
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
SCENARIO_DIR="$WS/src/robofetch_factory/config/scenarios"
CONFIG="$WS/config/run.yaml"
# System python3: PyYAML comes with ROS 2, so the config is readable before the venv exists.
RUNCFG=(python3 "$WS/scripts/run_config.py")

# Only used when the AI is switched off with model:= - otherwise the model decides everything.
DEMO_PLAN="PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100"

declare -A MODEL_TEXT=(
  [ns]="neuro-symbolic      rules + neural scorer (the thesis model)"
  [ns_symbolic_only]="symbolic only       the rules alone, no neural network"
  [ppo]="PPO                 deep reinforcement learning (comparison)"
  [rule]="rule-based          simple reference policy"
  [fallback]="no AI               built-in fallback rules only (AI service off)"
)
declare -A PLANNER_TEXT=(
  [NavfnDijkstra]="NavFn Dijkstra      wavefront over the whole map (Nav2 default)"
  [NavfnAStar]="NavFn A*            same costs, heuristic search"
  [Smac2D]="Smac 2D             cost-aware A*, smoothed; most precise arrival"
  [ThetaStar]="Theta*              any-angle, fewest turns (WP8 winner: fastest, least energy)"
  [SmacLattice]="Smac Lattice        diff-drive motion primitives; slow planning"
)

scenario_description() { grep -m1 '^description:' "$SCENARIO_DIR/$1.yaml" | sed 's/^description: *//'; }

index_of() {   # index_of value item...  -> 1-based position, or nothing
  local want="$1" i=1; shift
  for item in "$@"; do [ "$item" = "$want" ] && { echo "$i"; return; }; i=$((i + 1)); done
}

ask() {        # ask "question" default  -> echo answer
  local answer
  read -r -p "$1 [$2]: " answer
  echo "${answer:-$2}"
}

pick_from() {  # pick_from "question" current item...  -> echo the chosen item
  local question="$1" current="$2"; shift 2
  local items=("$@") default pick
  default=$(index_of "$current" "${items[@]}"); default=${default:-1}
  while :; do
    pick=$(ask "$question" "$default")
    if [[ "$pick" =~ ^[0-9]+$ ]] && [ "$pick" -ge 1 ] && [ "$pick" -le "${#items[@]}" ]; then
      echo "${items[$((pick - 1))]}"; return
    fi
    echo "  please enter a number between 1 and ${#items[@]}" >&2
  done
}

list_everything() {
  local i
  echo "Scenarios (src/robofetch_factory/config/scenarios/):"
  i=1; for s in "${SCENARIOS[@]}"; do
    printf "  %2d) %-18s %s\n" "$i" "$s" "$(scenario_description "$s" | cut -c1-90)"; i=$((i + 1))
  done
  echo
  echo "Decision models:"
  i=1; for m in "${MODELS[@]}"; do printf "  %2d) %s\n" "$i" "${MODEL_TEXT[$m]:-$m}"; i=$((i + 1)); done
  echo
  echo "Global path planners (Nav2):"
  i=1; for p in "${PLANNERS[@]}"; do printf "  %2d) %s\n" "$i" "${PLANNER_TEXT[$p]:-$p}"; i=$((i + 1)); done
}

# ------------------------------------------------------------------------------------ choices
CHOICES=$("${RUNCFG[@]}" choices) || exit 1
eval "$CHOICES"
read -r -a SCENARIOS <<< "$SCENARIOS"
read -r -a MODELS <<< "$MODELS"
read -r -a PLANNERS <<< "$PLANNERS"

# ---------------------------------------------------------------------------------- arguments
OVERRIDES=()        # key=value for the config loader
EXTRA_ARGS=()       # passed straight to the launch file
DRY_RUN=0; FORCE_ASK=0; NO_ASK=0; HAS_PLAN=0; NO_MODEL=0; SHIFT_S=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --list)        list_everything; exit 0 ;;
    --config)      CONFIG="$(realpath "$2")"; shift ;;
    --config=*)    CONFIG="$(realpath "${1#--config=}")" ;;
    --yes|-y)      NO_ASK=1 ;;
    --dry-run)     DRY_RUN=1 ;;
    --ask)         FORCE_ASK=1 ;;
    --headless)    OVERRIDES+=(view=headless); NO_ASK=1 ;;
    --no-build)    OVERRIDES+=(build=false) ;;
    scenario:=*)   OVERRIDES+=("scenario=${1#*:=}"); NO_ASK=1 ;;
    planner:=*)    OVERRIDES+=("planner=${1#*:=}"); NO_ASK=1 ;;
    seed:=*)       OVERRIDES+=("seed=${1#*:=}"); NO_ASK=1 ;;
    headless:=*)   [ "${1#*:=}" = true ] && OVERRIDES+=(view=headless) || OVERRIDES+=(view=gui); NO_ASK=1 ;;
    web:=*)        OVERRIDES+=("dashboard=${1#*:=}"); NO_ASK=1 ;;
    shift_s:=*)    SHIFT_S="${1#*:=}"; NO_ASK=1 ;;
    model:=)       NO_MODEL=1; NO_ASK=1 ;;
    model:=*)      OVERRIDES+=("model=${1#*:=}"); NO_ASK=1 ;;
    plan:=*)       HAS_PLAN=1; EXTRA_ARGS+=("$1"); NO_ASK=1 ;;
    *:=*)          EXTRA_ARGS+=("$1"); NO_ASK=1 ;;
    *)             echo "[run] unknown option '$1' (see the top of scripts/run.sh)"; exit 1 ;;
  esac
  shift
done

[ -f "$CONFIG" ] || { echo "[run] starter config not found: $CONFIG"; exit 1; }
LOADED=$("${RUNCFG[@]}" load "$CONFIG" "${OVERRIDES[@]}") || exit 1
eval "$LOADED"

# ------------------------------------------------------------------------------ interactive menu
if { [ "$CFG_ASK" = true ] && [ "$NO_ASK" -eq 0 ]; } || [ "$FORCE_ASK" -eq 1 ]; then
  if [ ! -t 0 ]; then
    echo "[run] the config asks questions but there is no terminal; use --yes"; exit 1
  fi
  echo
  echo "================ Autonomous factory robot ================"
  echo "  starter config: ${CONFIG#$WS/}   (Enter keeps the value in [brackets])"
  echo
  list_everything
  echo
  CFG_SCENARIO=$(pick_from "Scenario number" "$CFG_SCENARIO" "${SCENARIOS[@]}")
  CFG_MODEL=$(pick_from "Decision model number" "$CFG_MODEL" "${MODELS[@]}")
  CFG_PLANNER=$(pick_from "Path planner number" "$CFG_PLANNER" "${PLANNERS[@]}")

  echo
  echo "View:  1) Gazebo + RViz windows     2) headless (faster; watch on the dashboard only)"
  CFG_VIEW=$(pick_from "View" "$CFG_VIEW" gui headless)

  echo
  echo "Shift length in minutes of simulated time (0 = the full shift in params.yaml)."
  while :; do
    CFG_SHIFT_MINUTES=$(ask "Minutes" "$CFG_SHIFT_MINUTES")
    [[ "$CFG_SHIFT_MINUTES" =~ ^[0-9]+$ ]] && break
    echo "  please enter a whole number of minutes"
  done
fi

# ------------------------------------------------------------------------------ build launch args
[ -n "$SHIFT_S" ] || SHIFT_S="$((CFG_SHIFT_MINUTES * 60)).0"
LAUNCH_ARGS=("scenario:=$CFG_SCENARIO" "planner:=$CFG_PLANNER" "seed:=$CFG_SEED"
             "shift_s:=$SHIFT_S" "web:=$CFG_DASHBOARD")
[ "$CFG_VIEW" = headless ] && LAUNCH_ARGS+=('headless:=true') || LAUNCH_ARGS+=('headless:=false')
if [ "$NO_MODEL" -eq 1 ]; then
  # model:= (empty) means "no decisions at all, follow a script"; without an explicit plan the
  # demo sequence is used so the robot still does something visible.
  LAUNCH_ARGS+=('model:=' 'ai:=false')
  [ "$HAS_PLAN" -eq 0 ] && LAUNCH_ARGS+=("plan:=$DEMO_PLAN")
  MODEL_SHOWN="none (scripted plan)"
else
  LAUNCH_ARGS+=("model:=$CFG_MODEL")
  # The fallback rules run inside the robot's executor; the AI service is not needed for them.
  [ "$CFG_MODEL" = fallback ] && LAUNCH_ARGS+=('ai:=false')
  MODEL_SHOWN="$CFG_MODEL"
fi
LAUNCH_ARGS+=("${EXTRA_ARGS[@]}")

echo
echo "  config   : ${CONFIG#$WS/}"
echo "  scenario : $CFG_SCENARIO - $(scenario_description "$CFG_SCENARIO" | cut -c1-80)"
echo "  model    : $MODEL_SHOWN"
echo "  planner  : $CFG_PLANNER"
echo "  view     : $([ "$CFG_VIEW" = headless ] && echo headless || echo 'Gazebo + RViz')"
echo "  shift    : $SHIFT_S s simulated$([ "$SHIFT_S" = 0.0 ] && echo ' (full shift from params.yaml)')"
echo "  seed     : $CFG_SEED     dashboard: $CFG_DASHBOARD     build: $CFG_BUILD"

if { [ "$CFG_ASK" = true ] && [ "$NO_ASK" -eq 0 ]; } || [ "$FORCE_ASK" -eq 1 ]; then
  confirm=$(ask "Start? (y/n)" "y")
  [[ "$confirm" =~ ^[Yy] ]] || { echo "[run] cancelled"; exit 0; }
  save=$(ask "Save these choices as the defaults in ${CONFIG#$WS/}? (y/n)" "n")
  if [[ "$save" =~ ^[Yy] ]]; then
    "${RUNCFG[@]}" save "$CONFIG" "scenario=$CFG_SCENARIO" "model=$CFG_MODEL" \
      "planner=$CFG_PLANNER" "view=$CFG_VIEW" "shift_minutes=$CFG_SHIFT_MINUTES"
    echo "[run] saved"
  fi
fi

if [ "$DRY_RUN" -eq 1 ]; then
  echo; printf '[run] dry run, would launch:\n  ros2 launch robofetch_bringup mission.launch.py'
  printf ' %q' "${LAUNCH_ARGS[@]}"; echo; exit 0
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

if [ "$CFG_BUILD" = true ]; then
  echo "[run] 2/3  building the workspace ..."
  cd "$WS"
  source "$WS/venv/bin/activate"
  colcon build --symlink-install
else
  echo "[run] 2/3  skipping build (build: false)"
fi

echo "[run] 3/3  sourcing the workspace and launching ..."
source "$WS/install/setup.bash"

echo
echo "  The robot starts on the blue charger (south-west) and begins by itself after ~30 s."
echo "  Sections: A red (north-west), B yellow (north-east), C purple (south-east);"
echo "  delivery point green (south-centre)."
echo "  Watch it:  http://localhost:8000   (read-only dashboard: model, planner, live results)"
echo "  Results:   logs/<run_id>_mission_summary.yaml when the shift ends"
echo "  Stop:      Ctrl+C here, then ./scripts/stop.sh"
echo
exec ros2 launch robofetch_bringup mission.launch.py "${LAUNCH_ARGS[@]}"
