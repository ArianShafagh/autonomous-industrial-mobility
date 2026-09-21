# 15 — Configuration, tooling, tests and the development workflow

---

## 1. The configuration system

### 1.1 Files and responsibilities

| File | Contains | Edited by |
|---|---|---|
| `src/robofetch_factory/config/params.yaml` | **every tunable number**: time, robot, mission, factory | you |
| `src/robofetch_factory/config/scenarios/<name>.yaml` | a `name`, a `description`, and a partial override of `params.yaml` | you |
| `src/robofetch_factory/config/layout.yaml` | maze grid, POI zones, colours, physics step | you (then run the generator) |
| `src/robofetch_factory/config/poi.yaml`, `path_matrix.yaml` | generated poses and distances | `generate_world.py` only |
| `src/robofetch_nav/config/nav2_params.yaml` | Nav2 settings | you (navigation only) |
| `src/robofetch_gazebo/config/bridge.yaml` | Gazebo ↔ ROS topic bridge | rarely |

### 1.2 The full structure of `params.yaml`

```
time:          time_scale, shift_duration_s, seed
robot:         mass_kg
  battery:     capacity_wh, reserve_percent, charge_power_w, initial_percent
  energy:      idle_power_w, drive_wh_per_m, load_wh_per_m_per_kg, temp_loss_per_c, worn_extra_draw
  thermal:     ambient_c, heat_c_per_s, cool_per_s, payload_heat_per_kg, dock_cooling_factor, warn_c, max_c, resume_c
  wear:        condition_min_percent, per_wh, per_c_s_above_warn, initial_percent
  motion:      speed_m_s, drive_load, dwell_load
  handling:    max_payload_kg, load_time_s, unload_time_s
mission:
  navigation:  timeout_factor, timeout_min_s, arrival_tolerance_m, backup_distance_m, backup_speed_m_s, return_to_charger_on_failure
  objective:   value_per_unit_delivered, cost_per_lost_unit, cost_per_wh, cost_per_safety_violation, cost_per_second_idle (unused)
  simulation:  step_s, wait_slice_s, charge_targets_percent, nav_overhead_factor
  neurosymbolic: urgency_horizon_s, safety_margin_percent, hidden_sizes, training{episodes, rollout_horizon_s, rollout_samples,
                 discount_per_100s, exploration, epochs, batch_size (unused), learning_rate, validation_fraction, iterations, seed}
  rl:          total_timesteps, n_envs, n_steps, batch_size, learning_rate, gamma, gae_lambda, ent_coef, net_arch,
               use_action_masks, eval_seeds (unused), seed
factory:
  sections:    A|B|C: rate_per_hour, buffer_capacity, unit_mass_kg, initial_buffer  (+ any section_defaults key)
  section_defaults: production_cv, health_initial, wear_per_unit, degraded_below, min_rate_factor,
                    faults_per_hour, fault_wear_gain, repair_minutes, repair_cv
```

Every group is explained in its own section: robot (05), factory (06), navigation (07), objective and simulation (08), neurosymbolic (10, 11), rl (12).

### 1.3 Who reads what

| Consumer | Reads |
|---|---|
| `RobotParams.from_config` | `robot.*` (strict: missing **or unused** keys raise) |
| `build_sections` / `section_params` | `time.time_scale`, `time.seed`, `factory.*` |
| `FactorySim` | `mission.simulation`, `mission.objective`, `time.shift_duration_s`, `time.seed` |
| `SymbolicLayer` | `mission.neurosymbolic.urgency_horizon_s`, `safety_margin_percent` |
| `NeuroSymbolicPolicy` | `mission.objective` (for `symbolic_estimate`) |
| `train_ns.py` | `mission.neurosymbolic.hidden_sizes`, `training.*` |
| `train_ppo.py`, `PPOPolicy` | `mission.rl.*` |
| `mission_executor` | `mission.navigation`, `mission.simulation.charge_targets_percent`, `wait_slice_s`, `time.shift_duration_s` |

### 1.4 Validation rules (file 06 §11)

- Unknown scenario → error that lists the known ones.
- A key in a scenario or override that is not in `params.yaml` → `unknown parameter 'params.robot.battery.capacity_w' (not in params.yaml)`.
- Exception: any `section_defaults` key may appear under `factory.sections.<id>`.
- A `robot.*` key that no code reads → error. **This strictness only exists for `robot.*`.** Unused keys under `mission.*` (e.g. `cost_per_second_idle`, `eval_seeds`, `training.batch_size`) are not detected.

### 1.5 How to create a new scenario

```yaml
# src/robofetch_factory/config/scenarios/night_shift.yaml
name: night_shift
description: Long 8-hour shift with a slightly worn battery.
time:
  shift_duration_s: 28800.0
robot:
  battery: {capacity_wh: 18.0}
```

It is picked up automatically by `run.sh`, `evaluate.py`, `param_report.py --all`, the training scripts (if they list the directory) and the parametrised tests. Check it before use:

```bash
venv/bin/python scripts/param_report.py --scenario night_shift
```

---

## 2. `param_report.py`: judge a parameter set before running it

(Method and all-scenario output in file 06 §13.)

```bash
venv/bin/python scripts/param_report.py                                   # balanced, full report
venv/bin/python scripts/param_report.py --scenario high_demand
venv/bin/python scripts/param_report.py --set robot.battery.capacity_wh=30 --set factory.sections.B.rate_per_hour=45
venv/bin/python scripts/param_report.py --all                             # one line per scenario
```

`--set a.b.c=value` builds a nested override. Values are parsed as YAML, so numbers stay numbers and `[40, 70]` becomes a list. The same typo check applies.

HANDOVER shows how it was used to choose `high_demand`: 3× rates gave utilisation 1.11 (impossible), 2.5× gave 0.98, and **2×** gave 0.85, which was chosen as "hard but achievable".

---

## 3. All scripts and tools

### 3.1 `scripts/`

| Script | Purpose |
|---|---|
| `run.sh` | start the full autonomous system (interactive or with arguments), file 14 |
| `stop.sh` | stop everything; `--running`, `--check`, file 14 |
| `run_mission.sh` | one headless scripted mission with exit codes, file 07 |
| `sim.sh [soft\|headless]` | Gazebo + robot only (WSL GPU/software rendering choices) |
| `teleop.sh` | keyboard driving (`teleop_twist_keyboard`) |
| `goto.sh <poi\|x y [yaw]>` | send one `NavigateToPose` goal |
| `set_pose.sh [poi\|x y yaw]` | publish `/initialpose` for AMCL |
| `obstacle.sh add\|remove` | spawn or remove a box in the running world |
| `test_nav_recovery.sh [1\|2\|all]` | the two navigation-failure system tests |
| `check_nav.py` | WP1 system check: AMCL, planner-vs-matrix for all pairs, driven 7-leg tour → `logs/check_nav_*.csv` |
| `generate_world.py` | layout → world, map, POIs, matrix, pinch check (file 03) |
| `param_report.py` | parameter implications (above) |

### 3.2 `tools/`

| Tool | Purpose |
|---|---|
| `tools/ai/train_ns.py` | train the neural scorer (file 11) |
| `tools/ai/train_ppo.py` | train PPO (file 12) |
| `tools/ai/evaluate.py` | policies × scenarios × seeds (file 13) |
| `tools/ai/validate_against_gazebo.py` | replay a Gazebo mission in the fast simulator (file 08 §7) |
| `tools/nav/compare_planners.py` | WP8 planner benchmark (file 04 §6.2), **uncommitted** |

`tools/` contains a `COLCON_IGNORE` file, and every tool adds `src/robofetch_core`, `src/robofetch_factory` and `src/robofetch_ai` to `sys.path`, so tools run with `venv/bin/python` **without** sourcing ROS (except the nav tools, which need `rclpy`).

---

## 4. Tests: 161 tests, all passing

Run (without ROS sourcing, pure Python):

```bash
PYTHONPATH=src/robofetch_core:src/robofetch_factory:src/robofetch_ai \
  venv/bin/python -m pytest -q src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test
# 161 passed in 8.70s   (run on 2026-09-17)
```

| File | Tests | What it protects |
|---|---|---|
| `robofetch_core/test/test_robot_model.py` | 18 | config loading (missing/unused keys, scenarios reach the model), payload/heat/wear raise energy, idle power, trip = drive + idle, **realistic magnitudes** (5–40 Wh/km, 1–4 h runtime, 30–120 min charge), battery bounds, thermal steady state and docked cooling, overheating wears, `simulate_route` does not mutate, loaded-leg-only cost, simulated route = closed form |
| `robofetch_core/test/test_mission_plan.py` | 15 | parsing and invalid actions, destinations, pickup prediction uses matrix + load time, loaded delivery costs more, charge prediction reaches target, waiting charges at the charger and drains elsewhere |
| `robofetch_factory/test/test_factory_model.py` | 46 | user rates A/B/C, exact noise-free rate, time scale, noisy rate within 5 %, **step-size independence**, blocking and lost units, pickup unblocks, time-to-full accuracy, payload limit, never negative pickups, wear slows the line, fault stops and repair restores, worn machines fail sooner, seed reproducibility, independent streams, **every scenario loads and runs**, overrides touch only named keys, unknown scenario/key rejected, **every full buffer fits the robot** |
| `robofetch_ai/test/test_factory_sim.py` | 14 | conservation of units, energy = robot model, distance = matrix, payload, deliver, charge, wait, legal actions, empty section, battery-empty violation ends the shift, below-reserve recorded, seed reproducibility, shift length, episode speed |
| `robofetch_ai/test/test_factory_env.py` | 8 | Gymnasium `check_env`, observation shape/range, separate charge levels, mask = simulator, episode summary, illegal action penalty, same-seed trajectory, reward weights |
| `robofetch_ai/test/test_rule_policy.py` | 27 | delivers and never violates safety **in every scenario**, explains every decision **in every scenario**, blocked line first, charge vs deliver-first thresholds |
| `robofetch_ai/test/test_symbolic.py` | 11 | each hard rule with its wording, rules use the robot model, tier ordering, explanations, `allowed_by_tier` |
| `robofetch_ai/test/test_neurosymbolic.py` | 22 | **sabotaged network cannot break safety**, network only chooses inside the top tier, ablation modes differ, missing model → rules + note, **every decision explained in every scenario**, latency < 100 ms, save/load with normalisation, refuse foreign features |

**Testing styles used:**
- **Property-style tests**: invariants that must always hold (conservation, bounds, "never violates").
- **Parametrised over scenarios**: the scenario list is read from the directory, so new scenarios are tested automatically.
- **Adversarial tests**: the `AlwaysPicksTheWorst` scorer.
- **Sanity/magnitude tests** that catch physically unrealistic parameter values.
- **Statistical tests with tolerances** (noisy rates within 5 %, fault ratio ±25 % over 200 seeds).

**Not covered by automated tests:**
- ROS nodes (`factory_node`, `robot_state_node`, `mission_executor`), the decision service, the dashboard, launch files. These were verified manually in Gazebo and logged in HANDOVER.
- `generate_world.py` outputs (map size, POI clearance, pinch detection).
- `train_ns.py` / `train_ppo.py` (no smoke test in pytest).
- `evaluate.py` statistics.

---

## 5. Environment and build

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages venv
venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
source venv/bin/activate && colcon build --symlink-install && source install/setup.bash
```

`requirements.txt`: fastapi, uvicorn, jinja2, **numpy<2**, pillow, pyyaml, **torch (CPU)**, gymnasium, stable-baselines3, sb3-contrib, pandas, matplotlib, scipy, pytest, **setuptools<80**.

Recorded working versions (HANDOVER WP0): torch 2.14.0+cpu, gymnasium 1.3.0, stable-baselines3 2.9.0, numpy 1.26.4, setuptools 79.0.1.

**Only lower bounds and exclusions are pinned.** A fresh `pip install` in a year may pull incompatible majors. A lock file would fix that (§7).

---

## 6. Logs and the work log

### 6.1 `logs/` (git-ignored)

| Pattern | Written by | Content |
|---|---|---|
| `<run_id>_mission.csv` | mission_executor | one row per finished action, predicted vs measured |
| `<run_id>_mission_summary.yaml` | mission_executor | run totals, decisions, navigation stats, prediction error |
| `<run_id>_robot.csv` | robot_state_node | 1 Hz battery/temperature/condition/energy/distance/activity |
| `<run_id>_section_<A\|B\|C>.csv` | factory_node | 1 Hz section status |
| `<timestamp>_launch.log` | run_mission.sh | full launch output |
| `check_nav_*.csv` | check_nav.py | planner-vs-matrix and tour results |
| `nav_recovery_*.txt` | test_nav_recovery.sh | failure test report |
| `wp5_*.txt`, `wp7_*.log`, `wp8_*.log/csv` | manual runs | evidence for HANDOVER results |

At the time of writing: 27 runs with section logs, 23 mission CSVs, 17 summaries.

### 6.2 `HANDOVER.md`

The thesis work log. Each work package records **Done / Decisions / Problems and fixes / Results / Open issues / How to reproduce**, with real measured numbers. It is the primary evidence trail for the thesis and the best place to learn *why* something is the way it is. The current uncommitted state adds the paused WP8 section.

---

## 7. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Schema-validated config** | Only `robot.*` detects unused keys; types are not checked (a string `"5"` passes until arithmetic) | **Pydantic** models for the whole `params.yaml` (types, ranges like `0 < reserve_percent < 100`, unknown keys forbidden); or **Hydra/OmegaConf** for composition + CLI overrides + experiment sweeps |
| **Dependency lock** | Loose pins | `pip-tools` (`pip-compile` → `requirements.lock`) or `uv lock`; record the lock hash in training reports |
| **CI** | Tests run only by hand | GitHub Actions with `ros:jazzy` container: build, pytest, `evaluate.py --seeds 2 --scenarios balanced`, `generate_world.py` into a temp dir with an assert that nothing changed |
| **Tests for the generator** | Pinch detection is critical but untested | Feed the known-bad old grid (from HANDOVER) and assert it fails; assert POI clearance ≥ 0.45, matrix symmetry and triangle inequality |
| **ROS integration tests** | Nodes/launch untested | `launch_testing` + `launch_pytest`: start `factory_node` alone with `use_sim_time:=false`, call `/factory/B/pickup`, assert the response; start the service and POST `/decide` with a recorded state |
| **Pre-commit hooks** | Style and stale docstrings | `pre-commit` with `ruff` (lint + format), `mypy` on the pure-Python packages, `yamllint` |
| **Data versioning for models** | `ns_scorer.pt`/`ppo_policy.zip` are in git with metadata only inside the files | `DVC` or MLflow model registry, linking each model to its training config, data summary and code commit |
| **Documentation site** | Guides like this one plus API docs | `mkdocs-material` + `mkdocstrings` (docstrings are already rich) |
