# HANDOVER — Thesis work log

**Thesis:** Towards Autonomous Industrial Mobility: An AI Framework for Optimal Path Planning and Resource-Efficient Navigation
**Base:** copy of the RoboFetch workspace (ROS 2 Jazzy, Gazebo Harmonic 8.11, Nav2 + MPPI + AMCL).
**Repository:** new private repo `github.com/ArianShafagh/autonomous-industrial-mobility`, fresh git history (branch `main`).
**Old project:** stays unchanged at `github.com/ArianShafagh/robofetch` (last commit `db4fc40`) and in `~/robofetch_ws`.

Each work package (WP) gets one section: what was done, files, decisions, problems, real results, open issues.

---

## WP0 — Clean slate (2026-09-15)

### Done
- Git: the old history was dropped from this workspace and a **new repository with a fresh history** was started (the project is a different project). The old repo on GitHub is untouched. The old local `.git` was moved out of the workspace, not pushed anywhere.
  The 12 "modified" scripts in the initial git status were **only lost executable bits** from the copy (0 line changes) — restored with `chmod +x`.
- Deleted (untracked, not recoverable, not needed): `build/`, `install/`, `log/`, `logs/` (≈110 MB), broken `robofetch_venv/` (365 MB), `robofetch.db`, `report.zip`, 905 `*:Zone.Identifier` files, all `__pycache__/`, `.pytest_cache/`, `.vscode/` caches.
- Deleted old-project files (still available in the old `robofetch` repo): `report/`, `tools/report/` (UML/figures/plantuml), `docs/` (proposal, diagrams, ARCHITECTURE), old `HANDOVER.md`, `README.md`, `INSTALL.md`, `scripts/acceptance.py`, `scripts/toggle_ai.sh`. 86 tracked files removed in total.
- New `requirements.txt`, rewritten `.gitignore` (ignores `venv/`, build dirs, `tools/ai/results/`, `tools/ai/checkpoints/`, caches).
- New venv `venv/` (924 MB) created with `--system-site-packages` so ROS's `rclpy` is importable.

### Decisions
- **numpy pinned `<2`**: ROS Jazzy system Python packages are built against numpy 1.x; mixing 2.x in the venv risks ABI errors.
- **setuptools pinned `<80`**: pip pulled setuptools 84, which conflicts with colcon-core 0.21 (`requires setuptools<80`) and breaks `ament_python` builds when the venv is active.
- **CPU PyTorch** (RTX 2070 exists, but the models are small MLPs; PPO with MLP policies is faster on CPU and the CPU wheel is ~10× smaller).

### Problems
- `rm -rf robofetch_venv` left a phantom `lib64` entry inside the Bash sandbox (a sandbox view artefact, not a real file); removing it outside the sandbox worked.

### Results / verification
```
venv: torch 2.14.0+cpu, gymnasium 1.3.0, stable-baselines3 2.9.0, numpy 1.26.4, setuptools 79.0.1, rclpy import OK
colcon build --symlink-install  ->  Summary: 10 packages finished [31.3s]
pytest src/robofetch_core/test  ->  6 passed in 0.03s   (robot_model.py still intact)
```

### Open issues (handled in later WPs)
- Old code still references removed things and will be rewritten/deleted in its WP: `delivery.launch.py` hard-codes `~/robofetch_ws/robofetch_venv/bin/python` (WP7), `scripts/order.sh`, `tools/ml/*`, bridge tests, gripper (WP1/WP3/WP7).
- Current workspace path is `~/robofetch_ws_copy/robofetch_ws`; launch/scripts must derive paths instead of hard-coding them (WP7).

### How to set up the environment
```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages venv
venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
source venv/bin/activate && colcon build --symlink-install && source install/setup.bash
```

---

## WP1 — Factory maze environment (2026-09-15)

### Done
- **One source of truth for the world:** `src/robofetch_factory/config/layout.yaml` — an ASCII grid (0.5 m cells) of the hall plus POI definitions and colours.
- **`scripts/generate_world.py`** (replaces `generate_map.py`) produces from that file:
  - `robofetch_gazebo/worlds/factory_maze.sdf` — 14.5 × 10 m hall, 23 merged wall boxes, 3 coloured machines (1.4 m tall), floor pads for A/B/C, the delivery point and the charger (31 models);
  - `robofetch_nav/maps/factory_maze.pgm/.yaml` — 310 × 220 px at 0.05 m, map frame = world frame;
  - `robofetch_factory/config/poi.yaml` — robot poses for A, B, C, delivery, charger (snapped to ≥ 0.45 m clearance so the robot never stands against a machine);
  - `robofetch_factory/config/path_matrix.yaml` — shortest drivable maze path between every POI pair (8-connected Dijkstra on the map at ≥ 0.40 m clearance, then string-pulled).
  - A **pinch detector** that refuses to write a layout containing a gap the Nav2 planner would use but the robot cannot drive (see Problems).
- **New package `robofetch_factory`** (ament_python): installs the config, `layout.py` (`load_pois()`, `load_path_matrix()` — no ROS imports, reused later by the fast simulator) and `ros2 run robofetch_factory poi [name]`.
- **Environment layout:** A = north-west, B = north-east, C = south-east, **one delivery point** (south-centre), **one charger** (south-west, robot spawns there). A central block, several wall spurs and a dead-end region give alternative routes of different lengths.
- **Robot:** gripper link, gripper joint, the 6 DetachableJoint plugins and the fixed-joint-lumping workaround removed from the URDF/Gazebo xacro. Ground-truth pose publisher kept (used for measuring real distance and localisation error).
- **Bridge:** 24 parcel/gripper topic entries removed from `bridge.yaml`; cmd_vel, odom, tf, joint_states, scan, clock and ground-truth pose remain.
- **Launch:** `sim.launch.py` defaults to `factory_maze.sdf` and reads the spawn pose from `poi.yaml`; `navigation.launch.py` defaults to `factory_maze.yaml`.
- **Scripts:** `goto.sh` and `set_pose.sh` take POI names (`./scripts/goto.sh B`) or coordinates; `stop.sh` pattern updated for the new node names.
- **New system check `scripts/check_nav.py`:** sets the AMCL pose, compares Nav2 planner path lengths against the matrix for all 10 POI pairs, drives a 7-leg tour and logs success, time, ground-truth distance, goal error and AMCL error to `logs/check_nav_*.csv`.
- **Deleted:** `warehouse.sdf`, `warehouse.pgm/.yaml`, `generate_map.py`, `gripper_node.py`, `gripper.sh`.
- `venv/COLCON_IGNORE` and `tools/COLCON_IGNORE` added.

### Decisions
- **ASCII grid instead of rectangle lists:** the layout is readable at a glance and editable in seconds; walls are merged into few boxes automatically so Gazebo stays light.
- **Every passage ≥ 1.5 m (3 cells).** Lesson from the old project: anything under ~1.2 m wedges a 0.44 m robot with 0.35 m inflation.
- **Delivery point moved 5 m away from the charger** (first draft had them 1.4 m apart, which would make charging trips almost free and remove a real energy trade-off from the AI's decision).
- **Matrix is made symmetric** (min of A→B and B→A; greedy string-pulling differed by a few cm).

### Problems and fixes
1. **colcon picked up a test package inside `venv/`** (`my-test-package`, "Multiple top-level packages discovered") → `COLCON_IGNORE` in `venv/` and `tools/`.
2. **Stale `warehouse.*` files stayed in `install/`** after deleting the sources → clean rebuild of gazebo/nav/core packages.
3. **Hidden 0.5 m pinch in the first maze (the important one).** First run: Nav2 planned B→C = 10.47 m while the matrix said 22.89 m (−54 %), B→delivery −35 %, B→charger −14 %, and the robot drove **24.4 m instead of ~10 m** to reach B. Cause: a 0.5 m slot at grid row 13/col 22 between two wall segments. Nav2 only treats 0.22 m around obstacles as lethal, so the planner routed through the slot, the controller could not pass it, and the robot re-routed the long way — the same trap that broke the old project. Fix: widened the opening (rows 12, 14, 15 of the grid). Prevention: the generator now compares paths in "comfortable space widened by corner-hugging" against "any space ≥ 0.25 m clearance"; a route that is > 5 % shorter in the second space must go through a slot, and generation fails. Verified: fed the old layout back in, it flags exactly B↔C, B↔delivery, B↔charger — the three pairs Nav2 got wrong.
4. First pinch test compared raw lengths at two clearances and falsely flagged A↔B (−5.3 %) and A↔charger (−6.6 %) — those were only tighter corner cutting. Replaced by the slot-only comparison above.

### Results (final layout, headless Gazebo + Nav2, `logs/check_nav_20260915_114859.csv`)
Planner vs generated matrix (all 10 pairs):

| Pair | Nav2 planner (m) | Matrix (m) | Diff |
|---|---|---|---|
| A–B | 18.47 | 18.28 | +1.0 % |
| A–C | 15.78 | 15.56 | +1.4 % |
| A–delivery | 10.15 | 10.01 | +1.4 % |
| A–charger | 9.12 | 9.04 | +0.9 % |
| B–C | 9.34 | 9.24 | +1.1 % |
| B–delivery | 10.24 | 10.26 | −0.2 % |
| B–charger | 15.20 | 15.18 | +0.2 % |
| C–delivery | 6.30 | 6.30 | 0.0 % |
| C–charger | 11.36 | 11.27 | +0.8 % |
| delivery–charger | 5.20 | 5.10 | +2.0 % |

Worst |diff| **2.0 %** (target was ≤ 10 %).

Driven tour (ground truth from Gazebo):

| Leg | OK | Sim time (s) | Driven (m) | Matrix (m) | Goal error (m) | AMCL error (m) |
|---|---|---|---|---|---|---|
| charger → A | ✔ | 20.4 | 8.88 | 9.04 | 0.146 | 0.031 |
| A → delivery | ✔ | 27.3 | 10.29 | 10.01 | 0.143 | 0.010 |
| delivery → B | ✔ | 23.3 | 10.03 | 10.26 | 0.217 | 0.061 |
| B → delivery | ✔ | 26.9 | 10.21 | 10.26 | 0.186 | 0.029 |
| delivery → C | ✔ | 16.8 | 6.24 | 6.30 | 0.101 | 0.071 |
| C → delivery | ✔ | 17.6 | 6.36 | 6.30 | 0.154 | 0.068 |
| delivery → charger | ✔ | 15.1 | 5.26 | 5.10 | 0.071 | 0.036 |

**7/7 legs succeeded**, total driven 57.27 m vs matrix 57.27 m, 147.4 s sim time (**0.389 m/s average**), max goal error 0.217 m (inside Nav2's 0.25 m xy tolerance), max AMCL error **0.071 m**. `stop.sh --check`: no leftover processes.

### Open issues (for later WPs)
- `robot_model.EFFECTIVE_SPEED = 0.18 m/s` was measured in the old warehouse with grabbing; the maze measures **0.389 m/s**. Re-calibrate in WP3.
- ~~`robot_model.CAPACITY_WH = 11 Wh` gives ~31 m of empty driving~~ → **changed to 22 Wh (doubled) on the user's request.** New range: 62.9 m empty, 43.1 m carrying 2 kg, 29.3 m carrying 5 kg (at nominal temperature/condition). The full 57.3 m WP1 tour driven empty would use 91 % of the pack, so the robot still has to plan charging (15 % reserve). `pytest src/robofetch_core/test`: 6 passed. Load per product/section will be set in WP2 and may still require fine-tuning.
- `robofetch_interfaces/srv/Grab.srv` is kept until WP2 replaces it with the new messages (an interface package cannot be empty); `task_manager.py` and `delivery.launch.py` are legacy and stay non-functional until WP3/WP7.
- Optional extra challenge (moving obstacle / blocked corridor event) not added yet — planned for WP9 robustness tests.

### How to reproduce
```bash
venv/bin/python scripts/generate_world.py              # regenerate world/map/POIs/matrix
source /opt/ros/jazzy/setup.bash && source venv/bin/activate
colcon build --symlink-install && source install/setup.bash
ros2 launch robofetch_nav navigation.launch.py rviz:=false gz_extra:="-s --headless-rendering" &
venv/bin/python scripts/check_nav.py                   # after "Managed nodes are active"
./scripts/stop.sh
```

---

## WP2 — Live factory sections (2026-09-15)

### Done
- **`robofetch_factory/factory_model.py`** (pure Python, no ROS — the WP4 fast simulator will import the same code). A section = machine + output buffer:
  - **production:** each unit needs Gamma-distributed work (mean 1, cv `production_cv` = 0.2); step-size independent (units are produced event by event inside a step);
  - **buffer / BLOCKED:** full buffer stops the line; the production it would have made is counted as `lost_units`, with `blocked_time_s`;
  - **health / DEGRADED:** −0.15 % per unit; below 60 % the line slows to `0.5 + 0.5·health/100` of nominal;
  - **faults / FAULT:** hazard `faults_per_hour · (1 + 3·(1 − health/100))`, Gamma repair time (mean 45 factory min); repair services the machine to health 100;
  - `pickup(max_units, max_mass_kg)` for the robot; `snapshot()` with everything a monitor/AI needs, incl. **time_to_full_s** (sim seconds) and fault time remaining.
  - Separate random stream per section (`seed:section_id`), so one seed reproduces a whole shift.
- **Config:** `config/factory.yaml` (base) + `config/scenarios/` overrides merged by `load_config()`:
  - `balanced` — nominal; `one_hot_section` — B at 60 units/h, buffer 10; `fault_burst` — machines at health 55, 0.6 faults/h, 60 min repairs; `low_battery_start` — robot 35 % battery, buffers half full; `worn_robot` — robot condition 45 %.
  - Sections (user requirement kept): **A 20 units/h** (buffer 12, 0.4 kg), **B 30 units/h** (buffer 15, 0.3 kg), **C 5 units/h** (buffer 4, 1.2 kg). Every full buffer fits the robot's 5 kg payload in one trip.
  - Robot scenario fields for later WPs: initial battery/condition, `max_payload_kg` 5, load/unload dwell 6 s each.
- **Interfaces:** `Grab.srv` removed; new `msg/SectionStatus.msg` (status code+text, counters, buffer, rates, health, time-to-full, fault time left, lost units, blocked time) and `srv/Pickup.srv`.
- **`section_node`**: steps the model on the SIM clock (10 Hz), publishes `/factory/<id>/status` at 1 Hz, serves `/factory/<id>/pickup`, logs `logs/<run_id>_section_<id>.csv`.
- **`factory.launch.py`**: the three section nodes (`scenario:=`, `seed:=`, `use_sim_time:=`).
- **`factory_monitor`**: live terminal table of the three sections (read-only).
- **Tests:** `src/robofetch_factory/test/test_factory_model.py`, 27 tests.
- `stop.sh` pattern: `section_node`, `factory_monitor` added.

### Decisions
- **Everything the robot/AI sees is in simulation seconds;** rates stay "per factory hour". Factory time = sim time × `time_scale`, converted only inside the model.
- **`time_scale` = 5, shift = 3600 sim s (= 5 factory hours)** — chosen from a capacity calculation, not guessed (see Problems 2).
- **Each section is its own node** (each section "monitors itself", as in the scenario), sharing one run id so logs line up.
- Executable named `factory_monitor`, not `monitor`: `stop.sh` kills by process name and `monitor` is too generic.

### Problems and fixes
1. **Test `worn machines fail more often` failed (238 vs 205 faults, needed 1.5×).** Not a model bug: repair restores health to 100, so a worn machine is only worn until its first fault. Rewrote the test to measure time to the *first* fault: ratio matches the expected 2.8× (±25 %) over 200 seeds.
2. **Factory was overloaded at the first `time_scale` (20).** Characterisation showed every line blocking within ~90 s. Capacity calculation with the real energy model (`robot_model.energy_wh`, 22 Wh pack, measured 0.389 m/s, 12 s dwell, 1 %/s charging): one full-load trip delivery→A/B→delivery = ~64 s driving and **~49 % battery (~49 s charging)**; C = 44 s and 31 %. Robot utilisation needed for nominal demand: ts 20 → **3.12**, ts 8 → 1.25, ts 6 → 0.94, **ts 5 → 0.78**, ts 4 → 0.62. My first correction (ts 8, estimated 0.85) ignored charging and was still an overload — caught by doing the full calculation before accepting it. Final: **ts 5**.
3. `ros2 run … monitor` printed nothing under `timeout` (stdout buffered when piped) → `flush=True`.
4. A stale `monitor` executable stayed in `install/` after the rename → removed.

### Results
**Unit tests:** `pytest src/robofetch_factory/test` → **27 passed** (0.21 s). Covers: A/B/C rates = 20/30/5; noise-free section produces exactly its rate over 10 h; time_scale; noisy rate within 5 % over 100 h; identical results for step 2.0 s vs 0.1 s; blocking stops production and counts lost units exactly; pickup unblocks; predicted time-to-full matches actual (±0.5 s); payload limit (15 units × 0.4 kg with 5 kg limit → 12 taken); degraded rate factor; fault stops production for the repair time and restores health; worn machines fail 2.8× sooner; seed reproducibility; independent section streams; all 5 scenarios run a shift; overrides touch only named keys; every full buffer fits the robot.

**Live, wall clock (no Gazebo, ts 20 at the time):** 3 topics at exactly 1.000 Hz. After 26.3 factory min: B produced 13 (13.2 expected), A 7 (8.8 expected, within noise), buffers consistent (A 2+7 = 9; B 3+13−6 = 10). Pickup service: B with `max_mass_kg 2.0` → 6 units / 1.80 kg, 7 left; C → 1 unit / 1.20 kg.

**Live, Gazebo sim clock (`one_hot_section`):** section stamps follow `/clock` (10.1 → 125.0 s sim), factory time advanced **exactly 20.00×** sim time. B (60 units/h, buffer 10, started with 3) blocked after 7 units, 96.4 s blocked, **32.1 lost units = 96.35 s × 20 / 3600 × 60** (exact).

**Scenario characterisation — full 3600 s shift, no robot, 20 seeds each (final ts 5):**

| Scenario | Sec | First BLOCKED (s) mean / min | Potential units/shift | kg/shift | Faults/shift | Fault downtime (s) |
|---|---|---|---|---|---|---|
| balanced | A | 356 / 319 | 95.7 | 38.3 | 0.30 | 159 |
| balanced | B | 294 / 269 | 144.4 | 43.3 | 0.25 | 129 |
| balanced | C | 636 / 474 | 24.3 | 29.2 | 0.15 | 87 |
| one_hot_section | B | **85 / 76** | 289.3 | 86.8 | 0.25 | 128 |
| fault_burst | A | 944 / 432 | 55.2 | 22.1 | 2.35 | **1529** |
| fault_burst | B | 673 / 340 | 80.5 | 24.2 | 2.20 | 1562 |
| fault_burst | C | 1188 / 701 | 14.0 | 16.8 | 2.25 | 1391 |
| low_battery_start | A | 211 / 186 | 95.7 | 38.3 | 0.30 | 159 |
| low_battery_start | B | 172 / 147 | 144.5 | 43.4 | 0.25 | 128 |
| low_battery_start | C | 296 / 204 | 24.3 | 29.2 | 0.15 | 87 |

(`one_hot_section` A/C and `worn_robot` A/B/C are identical to `balanced` — those scenarios only change B or the robot.) Balanced demand ≈ **111 kg per shift**.

### Open issues
- **Energy per trip is heavy:** a full 4.8 kg trip costs ~49 % battery, so the robot must charge after roughly every second trip. This is what makes energy-aware decisions matter, but it must be re-checked in WP3 against the energy measured in Gazebo; if it proves unrealistic, `E_LOAD` or the payload limit is the knob.
- `robot_model.EFFECTIVE_SPEED` (0.18) still to be replaced by the measured 0.389 m/s (WP3).
- Legacy `task_manager.py` now fails at import (it used `Grab.srv`) — replaced by `mission_executor` in WP3; `delivery.launch.py` legacy until WP7.

### How to reproduce
```bash
source /opt/ros/jazzy/setup.bash && source venv/bin/activate
colcon build --symlink-install && source install/setup.bash
PYTHONPATH=src/robofetch_factory python -m pytest -q src/robofetch_factory/test
ros2 launch robofetch_factory factory.launch.py use_sim_time:=false scenario:=balanced &
ros2 run robofetch_factory factory_monitor
ros2 service call /factory/B/pickup robofetch_interfaces/srv/Pickup "{max_mass_kg: 5.0}"
./scripts/stop.sh
```

---

## WP2 revision — realistic numbers, all parameters in one config file (2026-09-15)

User request: make the numbers logical and put them in one config file so different variables can be tried to find the best scenario.

### Done
- **`src/robofetch_factory/config/params.yaml` is now the single file of tunable numbers** (replaces `factory.yaml`). Groups: `time`, `robot.{battery, energy, thermal, wear, motion, handling}`, `factory.{sections, section_defaults}`. Each value has its unit and its justification in a comment. Only the maze geometry stays in `layout.yaml` (it needs the world generator).
- **Scenarios** (`config/scenarios/*.yaml`) override any subset; now 6: `balanced`, `one_hot_section`, **`high_demand` (new)**, `fault_burst`, `low_battery_start`, `worn_robot`.
- **No numbers left in code:**
  - `robot_model.py` rewritten: `RobotParams.from_config(cfg)`, equations only. New physics: **idle electronics power** (energy is drawn while standing/waiting too), **charger power** (net = charger − idle), separate drive/idle/trip energy functions, `charge_time_s`, overheat time. The old Euclidean `route_legs` removed (maze path matrix is used instead).
  - `SectionParams` has no default values any more.
  - `robot_state_node` reads the scenario's `params.yaml`; also **fixed: it integrated battery/heat on wall-clock time**, now uses the simulation clock.
- **Typo protection both ways:** a scenario/override key that does not exist in `params.yaml` is rejected (`unknown parameter 'params.robot.battery.capacity_w'`); a `robot.*` key in `params.yaml` that no code reads is rejected too.
- **`scripts/param_report.py`** — run after any change. Shows range, runtime, charging time, full-load trip cost per section, robot utilisation (busy + charging), when each line blocks with no robot, and warnings (buffer heavier than payload, mission below reserve, overheating, utilisation > 1). `--set key=value` tries a value without editing files; `--all` compares all scenarios.

### The numbers (TurtleBot-class indoor AMR, matches this robot's size)
| Parameter | Old | New | Why |
|---|---|---|---|
| time_scale | 5 | **1.0** | real time for factory AND robot (the old scaling sped up production but not the robot) |
| battery | 22 Wh | 22 Wh | 11.1 V × 2 Ah pack (TurtleBot3: 19.98 Wh) |
| drive energy (empty) | 0.35 Wh/m | **0.005 Wh/m** | ≈7 W motor power at 0.39 m/s (old value = 490 W, 70× too high) |
| payload energy | 0.08 Wh/m/kg | **0.00116 Wh/m/kg** | = drive energy / robot mass (4.3 kg) |
| idle electronics | — (none) | **6 W** | computer ~4 W + lidar ~1.5 W + drivers |
| charging | 1 %/s (full in 100 s) | **25 W** charger (net 19 W) | ~1 C; reserve → full in 59 min |
| motor heating | 1.2 °C/s | **0.037 °C/s**, cooling 0.0011/s | steady 55 °C at full load, ~15 min time constant (old: 70 °C in a minute) |
| speed | 0.18 m/s | **0.39 m/s** | measured in WP1 |
| load / unload time | 8 s total | **30 s + 30 s** | realistic manual/automatic handling of up to ~10 parts |
| A / B / C rate | 20 / 30 / 5 units/h | 20 / 30 / 5 (user requirement) | unchanged |
| A / B / C buffer | 12 / 15 / 4 | **8 / 10 / 3** | 24 / 20 / 36 min of output |
| A / B / C unit mass | 0.4 / 0.3 / 1.2 kg | **0.5 / 0.4 / 1.5 kg** | full buffers 4.0 / 4.0 / 4.5 kg, all ≤ 5 kg payload |
| machine faults | 0.05/h, 45 min repair | **0.1/h (MTBF 10 h), 20 min repair** | |

### Problems and fixes
1. **My first drive-energy value was wrong by 3.6×.** I set 0.012 Wh/m and commented "≈4.7 W", but 0.012 Wh/m × 0.39 m/s = 16.8 W. The new sanity test (`test_realistic_magnitudes`: 5–40 Wh/km, 1–4 h driving runtime, 30–120 min charge) failed with a 0.96 h runtime → corrected to 0.005 Wh/m (1.69 h).
2. **`worn_robot` looked identical to `balanced` in the report** — the report computed trip energy for a new drive instead of the scenario's 45 % condition. Fixed; worn robot now shows 14.2 W vs 13.0 W while driving.
3. **First `high_demand` (3× rates) was impossible** (utilisation 1.11). Tried 2× and 2.5× with `--set` (0.85 and 0.98) → **2×** chosen: hard but achievable.
4. Stale `factory.yaml` symlink in `build/` broke the colcon build after the rename → clean rebuild of `robofetch_factory`.

### Results
`pytest src/robofetch_core/test src/robofetch_factory/test` → **52 passed** (18 robot model incl. config/typo/magnitude checks, 34 factory model).
`colcon build` → 10 packages finished. Section nodes start with the new values (`high_demand`: A 40 units/h, buffer 8, 0.5 kg …). `robot_state_node` with `low_battery_start` starts at 30 % and drains 0.0076 %/s standing (= 6 W on 22 Wh).

`param_report.py` (balanced):
```
ROBOT  battery 22 Wh, reserve 15 %, idle 6 W, driving 13.0 W total at 0.39 m/s
  energy per km (empty, incl. idle) 9.3 Wh; range empty 2372 m, with 5 kg 1460 m
  runtime: driving non-stop 1.69 h, standing idle 3.67 h; charging reserve -> full 59 min
FULL-LOAD TRIP delivery -> section -> delivery (incl. 60 s handling)
   A  10.01 m  4.00 kg  111 s  0.33 Wh  1.5 %   2.50 trips/h  10.0 kg/h
   B  10.26 m  4.00 kg  113 s  0.34 Wh  1.5 %   3.00 trips/h  12.0 kg/h
   C   6.30 m  4.50 kg   92 s  0.25 Wh  1.1 %   1.67 trips/h   7.5 kg/h
DEMAND: 29.5 kg/h; robot busy 21.4 %, energy 7.0 Wh/h -> charging 36.7 %  => UTILISATION 0.58
NO ROBOT: A blocks after 23.7 min, B 15.7 min, C 36.0 min
```
`param_report.py --all`:

| Scenario | Utilisation | Busy % | Charging % | kg/h | First BLOCKED A/B/C (min) | Warnings |
|---|---|---|---|---|---|---|
| balanced | 0.58 | 21.4 | 36.7 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |
| fault_burst | 0.58 | 21.4 | 36.7 | 29.5 | 35.0 / 28.0 / never | 0 |
| high_demand | 0.85 | 42.8 | 41.9 | 59.0 | 10.6 / 7.9 / 18.1 | 0 |
| low_battery_start | 0.58 | 21.4 | 36.7 | 29.5 | 11.6 / 10.0 / 24.5 | 0 |
| one_hot_section | 0.70 | 30.8 | 39.1 | 41.5 | 23.7 / 7.9 / 36.0 | 0 |
| worn_robot | 0.59 | 21.4 | 37.6 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |

### Finding worth noting for the thesis
With realistic numbers **most of the robot's energy is the electronics' idle power** (6 of 7 Wh per hour in `balanced`); one full-load trip costs only ~1.5 % of the battery. Energy efficiency is therefore mostly about **time** (fewer, better-batched trips, no pointless waiting away from the charger, charging while idle) rather than about metres of path. If the thesis should put more weight on path/drive energy, a heavier industrial AMR profile (e.g. 50 kg robot, 50–100 kg payload, larger motors) can be tried by changing only `params.yaml`.

### How to try variables
```bash
venv/bin/python scripts/param_report.py --all
venv/bin/python scripts/param_report.py --scenario one_hot_section --set robot.battery.capacity_wh=15 --set robot.handling.load_time_s=45
```

---

## WP3 — Mission executor (2026-09-15)

### Done
- **Action vocabulary** `robofetch_core/mission_plan.py` (pure Python, shared with the future simulator/AI): `PICKUP:<A|B|C>` (drive, load `load_time_s`, take what still fits the payload), `DELIVER` (drive, unload), `CHARGE:<%>` (drive to charger, charge to target), `WAIT:<s>` (waiting at the charger charges). `predict()` gives distance, time, energy and end battery from `robot_model` + the maze path matrix. The robot can collect from several sections before delivering.
- **`mission_executor`** (replaces `task_manager.py`): runs a scripted plan with no operator; waits for Nav2 to be ACTIVE, localises on the charger, drives with Nav2 (2 attempts per goal), calls `/factory/<id>/pickup`, dwells on the simulation clock, tells `robot_state_node` what it does and carries (`/robot/activity`), publishes **`TaskEvent`** on `/mission/events` (new message, predicted vs measured), writes `logs/<run_id>_mission.csv` and `_mission_summary.yaml`. Emergency stop kept (script-only `/robot/estop`): cancels the goal, brakes, ends the mission (`ended_by: emergency_stop`). The AI decision source replaces the scripted plan in WP7 without changing execution.
- **`mission.launch.py`** (replaces `delivery.launch.py`): Gazebo + Nav2 + factory + robot model + executor, `scenario:=`, `seed:=`, `plan:=`, `headless:=true`.
- **`scripts/run_mission.sh "<plan>" [scenario] [timeout]`**: one headless run, waits for the end, stops everything, exit code 0/1/2 — the basis for batch experiments later.
- **`factory_node`** replaces the three `section_node` processes (same per-section topics/services and CSVs).
- **Robustness:** physics step 1 ms → 4 ms (`layout.yaml: physics_step_s`); AMCL receives the spawn pose as initial pose from `poi.yaml` in `navigation.launch.py`; executor and `check_nav.py` wait for `/lifecycle_manager_navigation/is_active`; executor uses a single-threaded ROS executor.
- `robot_state_node`: `run_id`/`log_dir` parameters, log file `<run_id>_robot.csv`, logs `cumulative_charged_wh`.
- Deleted: `task_manager.py`, `section_node.py`, `delivery.launch.py`. Tests: `test_mission_plan.py` (15).

### Problems and fixes
1. **Launch arguments invisible in included launch files** (`launch configuration 'scenario' does not exist`): scoped groups had `forwarding=False` → `forwarding=True`.
2. **Nav2 bringup hung** — `failed to send response to /controller_server/change_state (timeout)`, the navigation lifecycle manager waited for ever, the executor waited for ever. Cause: every Python node on sim time processes every `/clock` message; at the 1 ms physics step (1000 Hz) the 3 section nodes + robot model used **~55 % CPU each**, load average **14 on 12 cores**, and lifecycle service replies got lost. Fix: one factory process + 4 ms physics step (250 Hz). Result: factory 26–30 %, robot model 28–34 %, load average **3.1**.
3. **Nav2 navigation bringup aborted** — `transform from base_footprint to map did not become available`. Navigation only activates once AMCL publishes `map→odom`, which needs an initial pose within ~60 s; WP1 and the first successful mission had only worked because a pose happened to arrive in time. Fix: AMCL gets the spawn pose at start (generated from `poi.yaml`), clients wait for `is_active`. Verified with the nav check started with **no** wait.
4. First navigation re-check at 4 ms showed paths through walls (A→charger 6.15 m = straight line) and a rejected goal — a symptom of problem 3 (the planner ran on an empty costmap), not of the physics step.
5. The launch directory vanished when `delivery.launch.py` (its only file) was removed with `git rm`, so the first `mission.launch.py` was never written → recreated.
6. `pkill -f mission_executor` killed my own shell (its command line contained the pattern) — never use `pkill -f` with a pattern that appears in the calling command.
7. Executor CPU with MultiThreadedExecutor(4) was 76 % → SingleThreadedExecutor: **43 %**.
8. Summary said "MISSION COMPLETE" after an emergency stop → now `MISSION ENDED BY EMERGENCY STOP` and `ended_by` in the summary; `run_mission.sh` returns 1.

### Results (headless Gazebo, all runs logged in `logs/`)
**Tests:** `pytest src/robofetch_core/test src/robofetch_factory/test` → **67 passed**.

**Navigation with 4 ms physics + AMCL initial pose** (`check_nav_20260915_131518.csv`, started with no wait): planner vs matrix worst **2.0 %** (identical to WP1), tour **7/7**, max AMCL error **0.077 m**, max goal error 0.221 m.

**Mission 1** — balanced, `PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100;WAIT:60` (run_20260915_125858):

| # | Action | Predicted m / s / Wh | Measured m / s / Wh | Result |
|---|---|---|---|---|
| 1 | PICKUP:B from charger | 15.2 / 69 / 0.19 | 15.0 / 63 / 0.18 | 2 units (0.8 kg) |
| 2 | PICKUP:A from B | 18.3 / 77 / 0.24 | 17.9 / 72 / 0.22 | 1 unit (0.5 kg) |
| 3 | DELIVER from A | 10.0 / 56 / 0.16 | 10.1 / 58 / 0.16 | 3 units delivered |
| 4 | PICKUP:C from delivery | 6.3 / 46 / 0.11 | 6.5 / 48 / 0.11 | nothing ready (C: 1 unit per 12 min) |
| 5 | DELIVER from C | 6.3 / 46 / 0.11 | 6.4 / 48 / 0.11 | 0 units |
| 6 | CHARGE:100 from delivery | 5.1 / 174 / 0.05 | 5.2 / 177 / 0.05 | 96.3 → 100 % |
| 7 | WAIT:60 at charger | 0 / 60 / 0 | 0 / 61 / 0 | charging while waiting |

7/7, 526 s, 61.2 m, 0.844 Wh drawn, 0.854 Wh charged. Total prediction error (non-charge): **duration −1.4 %, distance −0.2 %, energy −1.3 %**. Cross-checks: section B buffer 2 → 0 exactly at the pickup (t = 71 s); B produced 4 units in 466 s (3.9 expected); robot log shows `charging` only after arriving at the charger.

**Startup reliability** — 3 launches back-to-back, `PICKUP:C;DELIVER;CHARGE:100`: **3/3 complete** (205 s, 200 s, 205 s wall), measured values identical to ±1 s / ±0.1 m between runs; prediction error per run: duration −0.7/−1.0/−0.4 %, distance +1.0/+0.4/+1.0 %, energy −0.3/−0.5/−0.3 %.

**Long mission** — `low_battery_start` (30 % battery, half-full buffers), 12 actions incl. two charges and multi-section trips (run_20260915_165614):

| # | Action | Measured | Battery |
|---|---|---|---|
| 1 | PICKUP:B | 5 units, 15.0 m, 62 s | 29.9 → 29.1 % |
| 2 | PICKUP:A | 4 units, 18.1 m, 71 s | → 28.0 % |
| 3 | DELIVER | 9 units | → 27.1 % |
| 4 | CHARGE:40 | 565 s | → 40.0 % |
| 5 | PICKUP:C | 2 units (3.0 kg) | → 39.3 % |
| 6 | PICKUP:B | 5 units (payload limit reached, 2 left) | → 38.6 % |
| 7 | DELIVER | 7 units | → 37.6 % |
| 8 | WAIT:120 at delivery | 0.20 Wh idle | → 36.7 % |
| 9 | PICKUP:A | 6 units | → 36.1 % |
| 10 | PICKUP:B | 5 units | → 34.8 % |
| 11 | DELIVER | 11 units | → 33.9 % |
| 12 | CHARGE:45 | 493 s | → 45.0 % |

**12/12**, 1723 s sim, 123.6 m, delivered **A 10, B 15, C 2**, 2.09 Wh drawn, 5.41 Wh charged. Prediction error: duration −2.1 %, distance +0.2 %, energy −0.9 %. Charging 27.1 → 40 % in ~550 s = 2.84 Wh ≈ 18.6 W, matching the configured 19 W net.

**Emergency stop** — published `/robot/estop` 12 s into a drive to B: ground-truth speed **0.771 → 0.001 m/s within 2 s, 0.0 m/s for the next 10 s**; action FAILED, remaining plan not executed, summary written.

**CPU (during a mission):** Gazebo 78 %, executor 43 %, robot model 34 %, factory 30 %, bridge 27 %; load average 3.1.

### Open issues
- Nav2 success criterion is its 0.25 m xy tolerance; loading/unloading assumes the robot is "at" the section when Nav2 reports success. Good enough for virtual transport.
- A navigation failure currently moves to the next planned action; the AI (WP5–WP7) will decide what to do after a failure instead.
- Measured sim time runs faster than wall time in headless mode (speed 0.77 m/s by wall clock vs 0.39 m/s sim) — all our measurements use sim time, so results are unaffected.

### How to reproduce
```bash
source /opt/ros/jazzy/setup.bash && source venv/bin/activate && colcon build --symlink-install
./scripts/run_mission.sh "PICKUP:B;PICKUP:A;DELIVER;CHARGE:100" balanced 1200
cat logs/$(ls -t logs | grep mission_summary | head -1)
ros2 topic pub --once /robot/estop std_msgs/msg/String "{data: stop}"   # emergency stop
```

---

## WP3b — Navigation failure handling + manual stopping (2026-09-16)

User asked to fix problems before WP4: (a) how the robot handles navigation failures, (b) scripts stopping a running simulation behind the user's back.

### Navigation failure handling
- **Timeout per drive:** `max(timeout_min_s, timeout_factor × predicted drive time)` (60 s / 3.0 in `params.yaml → mission.navigation`). A stuck or oscillating robot can no longer hang a mission; the goal is cancelled.
- **Arrival verification (important finding):** Nav2 can report **SUCCESS without being at the goal** — when the goal pose is blocked, its planner (tolerance 0.5 m) plans to the nearest free spot and the controller "arrives" there. The first obstacle test showed PICKUP:C "succeeding" with a box sitting on C. Now, after every Nav2 success, the robot's **own localised pose** (AMCL, not Gazebo ground truth — a real robot has none) is compared with the requested pose: farther than `arrival_tolerance_m` (0.35 m; normal arrivals are ≤ 0.22 m) counts as a failure.
- **Recovery ladder:** attempt 1 → clear both costmaps → attempt 2 → back up 0.3 m (Nav2 BackUp behaviour) → attempt 3 → give up.
- **After giving up:** drive to the charger (the safe place); cargo stays on board (virtual, nothing is lost) and a later DELIVER still delivers it. If the charger is unreachable too, the robot **halts** (`_declare_stuck`) and the mission ends with `ended_by: navigation_stuck` — it needs a human.
- **Reporting:** every attempt and recovery step is written to the event/CSV `recovery` column, and the summary carries counters `drives, drive_attempt_failures, timeouts, recovered_after_retry, goals_abandoned, returned_to_charger`.
- **New test tooling:** `scripts/obstacle.sh add|remove <name> <poi|x y>` puts a real box into the running Gazebo world; `scripts/test_nav_recovery.sh [1|2|all]` runs the two failure scenarios end to end.

### Manual stopping (user request)
- No script stops a simulation automatically any more. `run.sh` and `run_mission.sh` **refuse to start** when something is already running (two simulations publish two `/clock` streams and both break) and print `./scripts/stop.sh`.
- `stop.sh --running` (new) prints how many simulation processes are alive; exit 0 if any.
- `run.sh` rewritten for the new system: launches `mission.launch.py` with a demo plan (`PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100`), accepts `--headless`, `--no-build` and any launch argument (`scenario:=`, `plan:=`). It previously still launched the deleted `delivery.launch.py` — this is what failed when the user ran it.
- Deleted the last user-ordering leftovers: `scripts/order.sh`, `scripts/order.py`.

### Results (headless Gazebo)
**Test 1 — box on section C's pickup pose**, plan `PICKUP:C;DELIVER`:
```
drive to C failed (Nav2 reported success but the robot is 0.62 m from C (goal blocked?)), attempt 1
reached C after recovery: ['attempt 1: ...0.62 m from C...', 'recovery: clear costmaps']
[1] PICKUP:C done | [2] DELIVER done | ended_by: plan_finished
navigation: drives 2, drive_attempt_failures 1, recovered_after_retry 1, goals_abandoned 0
```
False success detected, recovery worked, the mission continued.

**Test 2 — box on the charger after the robot left**, plan `PICKUP:A;CHARGE:100`:
```
[1] PICKUP:A done: took 1 units (0.50 kg)
drive to charger failed (timeout after 70 s), attempt 1
drive to charger failed (Nav2 aborted), attempt 2          (after clear costmaps)
drive to charger failed (timeout after 70 s), attempt 3    (after back up)
NAVIGATION STUCK: the robot cannot reach the charger. Halting - needs a human.
ended_by: navigation_stuck
navigation: drives 2, drive_attempt_failures 3, timeouts 2, goals_abandoned 1
```
The full ladder ran, the timeout worked, and the robot stopped instead of driving for ever.

`pytest src/robofetch_core/test src/robofetch_factory/test` → 67 passed.

### Open issues
- After a recovered arrival at a blocked section the robot may still be ~0.3 m off; loading is allowed within `arrival_tolerance_m`. Tighten only if a blocked pickup point ever has to be refused outright.
- A blocked route is currently only visible to the AI as a failed action (WP5–WP7 will decide what to do next: retry later, pick another section, charge).

---

## WP4 — Fast simulator, policy interface and rule-based reference (2026-09-16)

### Done
- **`robofetch_ai/env/factory_sim.py`** — the whole factory without Gazebo or ROS. It executes the **same actions** (`mission_plan`), moves the robot with the **same** energy/thermal/wear model (`robot_model`), uses the **same** maze path lengths (`path_matrix.yaml`) and runs the **same** production sections (`factory_model`); only Nav2 + physics are replaced by "distance / speed". One shift: **0.07 s** (Gazebo: ~30 min).
  - `state()` gives a decision model exactly the facts the live system publishes (section status + robot telemetry) plus distances and "units that still fit".
  - `legal_actions()` refuses pointless actions (pickup from an empty section, deliver with an empty load, charge when already full at the charger) — the mask shared by the rules, the neuro-symbolic layer and the RL agent.
  - Safety violations recorded: `battery_empty`, `below_reserve`, `overheated`.
  - `summary()`: delivered units (per section), lost production, energy, **Wh per unit**, distance, charge trips, min battery, violations, score.
- **`robofetch_ai/env/factory_env.py`** — Gymnasium wrapper for the PPO comparison (WP6): 6 discrete actions (PICKUP A/B/C, DELIVER, CHARGE, WAIT), `action_masks()`, ~28-value observation, reward = the configured objective.
- **`robofetch_ai/policies/base.py`** — `Policy.decide(state, legal_actions, sim) -> Decision(action, explanation, scores)`. One interface for rule-based, neuro-symbolic and PPO, used by the fast simulator AND later by the live decision service.
- **`robofetch_ai/policies/rule_based.py`** — the reference line and the live fallback: charge when the battery only just covers getting home; serve BLOCKED sections first, then the one that blocks soonest; fill the load on the way; deliver what you carry; wait at the charger (where waiting is free). Every decision returns a sentence explaining itself.
- **Objective in `params.yaml`** (`mission.objective`): +1 per delivered unit, −1 per lost unit, −0.5 per Wh, −20 per safety violation. Training and reporting therefore optimise the same thing. `mission.simulation` holds the step size, the WAIT slice (60 s), the CHARGE target (90 %) and a `nav_overhead_factor`.
- **`tools/ai/evaluate.py`** — policies × scenarios × seeds → `tools/ai/results/episodes_*.csv` + a table with means and a 95 % bootstrap CI on the score (paired seeds across policies).
- **`tools/ai/validate_against_gazebo.py`** — replays a finished Gazebo mission's actions in the fast simulator and compares distance/time/energy per action and in total; fails above 15 %.
- Deleted the old classifier service (`robofetch_ai/service.py`, `models/model.joblib`) and `tools/ml/`. The mission summary now records scenario, seed and plan.

### Results
**Fast simulator vs the real Gazebo run** (`run_20260915_125858`, 7 actions):

| Total | Gazebo | Fast sim | Diff |
|---|---|---|---|
| distance | 61.17 m | 61.17 m | +0.0 % |
| duration | 526.4 s | 529.1 s | +0.5 % |
| energy | 0.844 Wh | 0.849 Wh | +0.6 % |

Worst per-action difference: PICKUP:B 62.6 s vs 68.9 s (+10 %). **Worst total 0.6 %** (target ≤ 15 %), so what a model learns in the fast simulator transfers to the robot.

**Rule-based reference, 20 seeds per scenario (120 episodes in 5.8 s):**

| Scenario | Score | Delivered | Lost | Energy Wh | Wh/unit | Distance m | Charge trips | Min battery % |
|---|---|---|---|---|---|---|---|---|
| balanced | 46.7 | 49.1 | 0.0 | 4.86 | 0.10 | 343 | 9.1 | 96.1 |
| one_hot_section | 74.0 | 77.3 | 0.0 | 6.44 | 0.08 | 451 | 11.2 | 93.6 |
| high_demand | 94.4 | 98.2 | 0.02 | 7.53 | 0.08 | 495 | 8.7 | 85.6 |
| fault_burst | 31.3 | 33.0 | 0.0 | 3.41 | 0.10 | 243 | 7.2 | 96.5 |
| worn_robot | 46.5 | 49.1 | 0.0 | 5.21 | 0.11 | 343 | 9.1 | 95.7 |
| **low_battery_start** | **9.6** | 32.3 | **21.6** | 2.23 | 0.07 | 140 | 2.0 | 27.1 |

No safety violations anywhere. The interesting case is `low_battery_start`: the rule policy charges to 90 % first, which costs ~42 min of the 60 min shift, and **21.6 units of production are lost**. A better policy would charge partially, or serve the nearest line first — exactly the decision the thesis is about, and a clear gap for the neuro-symbolic model and PPO to close.

**Tests:** `pytest src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test` → **103 passed** (36 new: unit conservation, energy = robot-model energy, distances = path matrix, payload limit, charge/wait behaviour, legal actions, violations, seed reproducibility, episode speed; Gymnasium checker, action mask vs simulator, reward = objective weights; rule policy delivers, never violates safety and explains every decision in all 6 scenarios).

### Problems and fixes
1. A rule-policy test demanded charging at 17 % battery; the policy correctly delivered first (it needed 15.4 %). The test was wrong and now pins both cases: charge at 15.6 %, deliver at 17 %.
2. `Outcome.violations` was never filled, so per-action rewards ignored violations → violations are now diffed per action.

### How to run
```bash
venv/bin/python tools/ai/evaluate.py --seeds 20               # all scenarios, rule policy
venv/bin/python tools/ai/evaluate.py --scenarios balanced --trace   # see the decisions
venv/bin/python tools/ai/validate_against_gazebo.py           # newest Gazebo mission vs fast sim
```

---

## WP5 / WP6 — Neuro-symbolic decision model and the PPO comparison (2026-09-16)

### What was built
**Neuro-symbolic model** (`robofetch_ai/policies/`):
- `symbolic.py` — the rules, in code, in words.
  - **Hard rules (forbid, never overridable):** battery must still cover "reach the charger + reserve" *after* the action (checked by forward-simulating the route with `robot_model`), motors must stay below `max_c`, a pickup must fit the payload, and a robot below `condition_min` may only charge or wait.
  - **Priority rules (rank):** tier 2 = unblock a stopped line, tier 1 = prevent one from stopping, tier 0 = routine.
  - Every verdict carries its reason, so a decision can always be explained.
- `neural.py` — `ActionScorer`, a 2×64 MLP (~6 k parameters) scoring ONE candidate action; feature normalisation is stored with the weights; loading a model trained on different features is refused.
- `features.py` — 27 features per candidate action, plus `symbolic_estimate()`: the obvious part of the value (units moved × value − energy × cost). **The network only learns the correction to this**, which is what keeps it sane when unsure.
- `neurosymbolic.py` — the model: hard rules → top priority tier → neural ranking inside that tier → decision + explanation. Modes `full`, `neural` (ablation: no rules), `symbolic` (ablation: no network, ranks by energy per unit). Without a trained model it runs symbolic and says so.
- **Charging became a decision**: `CHARGE 40 / 70 / 100 %` instead of one fixed target, so "top up briefly and keep the lines running" is expressible.

**PPO comparison** (`policies/rl_ppo.py`, `tools/ai/train_ppo.py`): MaskablePPO (sb3-contrib) on the same environment, actions and objective. Two honest differences: no symbolic layer (it can only *learn* safety) and no explanation beyond the action probability. 300 k steps = 7.9 min on CPU.

**Training** (`tools/ai/train_ns.py`): no human labels. At each decision the simulator is cloned, each candidate executed, and the shift played on by a reference policy for `rollout_horizon_s`; the discounted objective collected is what that action was worth. Policy iteration over rounds (round 1 looks ahead with the symbolic policy, later rounds with the model trained so far), states walked by the current model (DAgger), only candidates that survive the symbolic layer are labelled, and the round with the best validation score is kept.

### The three fixes that made the model work (each one measured)
1. **Residual learning.** First version: the network predicted the whole value → it lost to the plain rules (low battery 25.5 vs 54.0 for symbolic-only). Learning only the correction to `symbolic_estimate` raised oracle agreement from ~40 % to 60 %.
2. **Best-round selection.** The last round was being saved even when worse; now each round is scored on unseen validation shifts and the best is kept.
3. **Ranking instead of regression (the decisive one).** The look-ahead value carries ±2.5 units of noise while two candidates typically differ by < 1 unit, so regressing values spent the network on noise. Training it to pick the better candidate (softmax cross-entropy over the candidates of each decision, small value term kept for readable scores) took the validation score from 56.8 to **59.7** and turned a model that lost to the rules into one that leads in 4 of 6 scenarios.
   - A change that did NOT help is recorded too: walking the shifts with the model's own states (DAgger) alone made it slightly worse (56.8 vs 58.3). It was kept because it is methodologically right, but the gain came from the ranking loss.

### Results — 20 seeds per scenario, fast simulator (`logs/wp5_eval6.txt`)
Score = delivered units − lost production − 0.5 × Wh − 20 × safety violations.

| Scenario | rule | **neuro-symbolic** | symbolic only | PPO |
|---|---|---|---|---|
| balanced | 46.8 | **47.9** | 46.3 | 46.8 |
| one_hot_section | 72.8 | **74.8** | 72.9 | 74.4 |
| high_demand | 93.2 | **96.9** | 96.7 | 96.4 |
| fault_burst | 31.0 | **32.0** | 30.7 | 31.5 |
| low_battery_start | −4.9 | 49.5 | **54.0** | 48.4 |
| worn_robot | **46.6** | 43.9 | 45.4 | 46.4 |

**Safety and efficiency**

| Policy | Episodes with safety violations | Lost production | Energy (balanced) | Explains decisions |
|---|---|---|---|---|
| **neuro-symbolic** | **0 / 120** | **0.00 everywhere** | 8.0 Wh | yes, every decision |
| symbolic only | 0 / 120 | ≤ 0.52 | 8.4 Wh | yes |
| rule | 0 / 120 | ≤ 27.8 | 4.9 Wh | yes |
| neural only (ablation) | **20 / 20** in low battery | ≤ 28.5 | 8.4 Wh | no (score only) |
| PPO | **5 / 20** in low battery | ≤ 0.40 | 10.1 Wh | no (probability only) |

Headline for the thesis: **the neuro-symbolic model matches or beats PPO on throughput while using ~20 % less energy, never violates safety, and explains every decision** — and the ablations show where that comes from: remove the rules and the same network drains the battery to 0 % in every hard shift; PPO, which can only learn safety, violates it in a quarter of them.

### Tests
`pytest src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test` → **131 passed**, including:
- every hard rule has a test that constructs the situation and checks the refusal AND its wording;
- a **deliberately sabotaged network** (always prefers the most dangerous action) runs a whole shift with zero violations and a battery that never drops below the reserve — the guarantee does not depend on what the network learnt;
- the ablations really differ (neural-only ignores the rules), a missing model falls back to rules and says so, decision latency < 100 ms.

### Open issues / next steps for the model
- `worn_robot` (43.9) is below the rules (46.6), and `low_battery_start` is still better with symbolic-only (54.0 vs 49.5).
- Policy iteration does not pay off yet: round 1 was best in the final run; rounds 2–3 got worse.
- PPO is trained masked; the `ppo_unmasked` ablation (how much of the benefit is simply knowing the rules) is implemented but not yet run.

### How to reproduce
```bash
venv/bin/python -u tools/ai/train_ns.py                  # ~40 min, 3 rounds, keeps the best
venv/bin/python -u tools/ai/train_ppo.py --threads 3     # ~8 min
venv/bin/python tools/ai/evaluate.py --policies rule ns ns_neural_only ns_symbolic_only ppo --seeds 20
venv/bin/python tools/ai/evaluate.py --policies ns --scenarios low_battery_start --seeds 1 --trace
```

---

## WP7 — Live integration: the AI drives the robot, and a dashboard shows it (2026-09-16)

### Done
- **Decision service** (`robofetch_ai/service.py`, port 8001, its own process): `POST /decide {model, scenario, state}` → `{action, explanation, scores, latency_ms}`; `GET /health`, `GET /models`. Models: `ns`, `ns_symbolic_only`, `ns_neural_only`, `ppo`, `rule` — the same objects evaluated in the fast simulator, so what the thesis measures offline is what drives the robot. A failing model returns an error instead of taking the robot down.
- **`robofetch_ai/env/live_state.py`**: presents the REAL system (section messages + robot telemetry) through the same interface as `FactorySim` (`state()`, `legal_actions()`, `p`, `matrix`). The deciding code is literally the same in simulation and on the robot — no second implementation to drift.
- **Autonomous executor**: with `model:=…` the executor asks the service before every action and executes the answer with Nav2 until the shift ends; `plan:=…` still runs a scripted sequence for reproducible tests. It publishes `/mission/decision` (model, action, explanation, scores, latency, fallback flag) and logs decision statistics in the run summary.
- **Fallback** (`robofetch_core/fallback_policy.py`): ~40 lines, no dependencies, deliberately placed in `robofetch_core` so it cannot depend on the thing that failed. Used whenever the service does not answer; every such decision is marked `fallback` in the log and on the dashboard.
- **Read-only dashboard** (`robofetch_bridge/app.py` + `ros_link.py`, port 8000): section cards (buffer bar, status, produced/collected, rate, health, time-to-full, lost production), robot card (battery, activity, payload, temperature, condition, position, distance, energy), shift totals, the AI's decisions with their reasons and latency, and the actions carried out with **predicted vs measured** time/distance/energy. `GET /api/state` returns the same data as JSON (the source for thesis figures). **No publisher, no buttons, no login** — the only intervention is `./scripts/stop.sh` at the console.
- Deleted the last of the old web tier: `admission.py`, `predictor.py`, `db.py`, the ordering `app.py`/`ros_link.py`, all 11 old templates and the old bridge tests.
- `mission.launch.py` now starts the decision service too, with arguments `model:=`, `ai:=false` (run on the fallback rules), `shift_s:=`.

### Results
**Full autonomous shift in Gazebo** (`model:=ns`, balanced, 900 s shift, `run_20260916_170805`):

| | |
|---|---|
| actions | **17 / 17 succeeded**, 0 failed |
| decisions | 17 asked, **17 answered by the model, 0 fallbacks** |
| delivered | **13 units** (A 4, B 8, C 1) |
| driven / energy | 170.1 m / 2.505 Wh, battery 100 → 88.6 % |
| navigation | 17 drives, 0 failures, 0 recoveries, 0 timeouts |
| prediction error | duration −0.5 %, distance −0.1 %, energy −0.2 % |

Example live decision: `PICKUP:A — [routine] A has 2 units waiting; neural score −0.89, chosen over PICKUP:B by +0.34`, answered in **7 ms**.

**Fallback under failure** (`logs/wp7_fallback.log`): the decision service was killed mid-shift. The model answered 2 decisions, then the executor logged `decision service unavailable (Connection refused); using the fallback rules` and finished the shift on its own rules (5 decisions, e.g. `fallback: carrying 4 units, nothing to collect`). The robot never stopped and never waited for a human; the summary reports `answered_by_model: 2, fallback: 5`.

**Dashboard against the running system**: `/health` → connected, 3 sections; the page showed live buffers (A 2/8, B 1/10, C 0/3), battery 89.2 %, 13 produced / 1 delivered / 0 lost, 166 m, 2.4 Wh, and the decision table with reasons and latency.

**Tests**: `pytest src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test` → **131 passed**; `colcon build` → 10 packages.

### Problems and fixes
1. **Decision service died at launch**: it was started with the system `python3` (no FastAPI) because the launch file resolved the workspace with `abspath`, which does not follow the `--symlink-install` symlink from `install/` into `src/` → `realpath`.
2. **`shift_s:=900` rejected**: an integer where the node declares a double → `ParameterValue(..., value_type=float)`.
3. **Dashboard returned 500 while the JSON API worked**: Starlette's `TemplateResponse` now takes `(request, name, context)`; passing the name first made it treat the context dict as the template name ("unhashable type: dict").
4. **Infinite scores could not be sent**: symbolic-only scores "moves no units" as −infinity, which JSON cannot encode → reported as `null`, with a `json_safe()` guard in the service.
5. `pkill -f "uvicorn robofetch_ai"` killed the calling shell again (the pattern matched its own command line). Kill by port instead: `fuser -k 8001/tcp`.

### How to run the whole thing
```bash
./scripts/run.sh                          # Gazebo + RViz + AI (model ns) + dashboard-ready topics
ros2 launch robofetch_bringup mission.launch.py headless:=true model:=ppo scenario:=high_demand
venv/bin/python -m uvicorn robofetch_bridge.app:app --port 8000     # dashboard at localhost:8000
fuser -k 8001/tcp                         # "the AI is down": the robot continues on fallback rules
./scripts/stop.sh                         # the only way to stop the system
```

### Open issues
- The dashboard is not started by `mission.launch.py` yet (run it manually); it should become a launch argument.
- Decision latency in the summary includes the first request, which loads the model (~1.5 s); warm requests are 3–7 ms.
- The maze map with the robot's position is not drawn on the page yet (planned for the evaluation work).

---

## Scenarios and interactive start (2026-09-16)

User request: try several scenarios (heavy load, balanced, low battery and other realistic ones) and choose the mode at run time from the run script.

### Done
- **Six new scenarios** (12 in total), each checked with `param_report.py` before being accepted:

| Scenario | What changes | Utilisation | Lines block after (A/B/C, no robot) |
|---|---|---|---|
| `heavy_load` | double rates, heavier parts, robot starts at 50 % | **1.03** (over capacity on purpose) | 7.6 / 6.0 / 11.9 min |
| `small_buffers` | buffers of 3 / 4 / 1 units | 0.99 | 5.7 / 6.1 / 12.2 min |
| `aged_battery` | 12 Wh pack (55 % of new), 15 W charger | 0.99 (charging 77 % of the time) | 23.7 / 15.7 / 36.0 min |
| `heavy_parts` | units ~2× heavier, fewer per trip | 0.75 | 11.6 / 8.0 / 24.5 min |
| `section_breakdown` | only B failing: health 35 %, 2 faults/h, 25 min repairs | 0.58 | 23.7 / never / 36.0 min |
| `hot_factory` | 38 °C hall, weaker cooling (motor limit matters on long heavy work) | 0.58 | 23.7 / 15.7 / 36.0 min |

  Existing: `balanced`, `high_demand`, `low_battery_start`, `one_hot_section`, `fault_burst`, `worn_robot`.
- **Config loader:** a single section may now override any `section_defaults` key (needed for `section_breakdown`); every other unknown key is still rejected as a typo.
- **`scripts/run.sh` is interactive** when started without arguments: it lists the 12 scenarios with their descriptions and the 5 decision modes, then asks for scenario, model, view (Gazebo + RViz or headless) and shift length, shows a summary and asks for confirmation. Invalid input is re-asked. `--list` prints the choices without starting anything. With arguments it stays non-interactive for repeat runs (`scenario:=… model:=… shift_s:=… --headless --no-build`).
- **Decision modes:** `ns` (neuro-symbolic), `ns_symbolic_only`, `ppo`, `rule`, and **`fallback` = no AI**: the robot runs on its built-in rules inside the executor and the AI service is not started (not reported as a failure, unlike a service that dies mid-shift). `model:=` with a `plan:=` still runs a scripted sequence.
- **Dashboard started by the launch file** (`web:=true` by default), so one command gives the robot, the AI and the monitor.

### Results
- `pytest` (core, factory, ai) → **161 passed** — the scenario-parametrised tests now also cover the six new scenarios (load, payload fits, safe and explained shift for every scenario).
- Interactive menu tested with a simulated terminal, including invalid answers (99 as scenario, 3 as view, "abc" as minutes) and cancelling.
- Real launch `./scripts/run.sh scenario:=hot_factory model:=fallback --headless --no-build shift_s:=180.0`: scenario `hot_factory` loaded, 3 decisions all by the built-in rules (`fallback: section B fills in 928 s` …), 0 "service unavailable" warnings, AI service correctly not running.
- One-command check (`mission.launch.py headless:=true`): dashboard `/health` and AI `/health` both up, first decision `PICKUP:B - B has 2 units waiting; neural score +4.67`.

### How to use
```bash
./scripts/run.sh                                 # asks: scenario, model, view, minutes
./scripts/run.sh --list                          # show scenarios and models
./scripts/run.sh scenario:=heavy_load model:=ppo shift_s:=900.0 --headless
./scripts/stop.sh                                # stopping is always manual
```

---

## WP8 — Global path planner comparison (done 2026-09-17, awaiting approval)

### What was done
- `nav2_params.yaml`: five planners loaded side by side — `NavfnDijkstra` (old default), `NavfnAStar`, `Smac2D`, `ThetaStar`, `SmacLattice` (diff-drive motion primitives, 5 cm / 0.5 m turning radius). The behaviour tree's `GridBased` is set by the new launch argument `planner:=` (navigation.launch.py, passed through mission.launch.py).
- `tools/nav/compare_planners.py`: the planner is isolated — each planner computes the path, the SAME MPPI controller follows it (FollowPath). Phase 1 plans all 20 ordered POI pairs (success, planning time, length vs the generated shortest path, total turning, clearance from walls); phase 2 drives the 7-leg tour per planner (time, ground-truth distance, energy from the robot model, arrival error). `--drive-repeats N` rotates the planner order each repeat and reports mean ± 95 % CI plus a paired t-test against NavfnDijkstra; results are saved after every repeat.

### Results so far
**Phase 1 — planning, 20 routes × 3 repeats (300/300 planned):**

| Planner | Plan ms mean / p95 | Length vs shortest | Turning rad | Min clearance m |
|---|---|---|---|---|
| NavfnDijkstra | 15.3 / 36.0 | +1.0 % | 7.40 | 0.34 |
| NavfnAStar | 18.6 / 35.6 | +2.3 % | 11.59 | 0.32 |
| Smac2D | 9.2 / 20.0 | +0.7 % | 4.88 | 0.28 |
| **ThetaStar** | **7.9 / 18.2** | **−0.6 %** | **4.07** | 0.32 |
| SmacLattice | 129.6 / 680.0 | +6.8 % | 5.21 | 0.29 |

**Phase 2 — one tour per planner (35/35 legs driven):**

| Planner | Time s | Driven m | Energy Wh | Wh/km | Turning rad | Arrival error m |
|---|---|---|---|---|---|---|
| NavfnDijkstra | 151.5 | 57.17 | 0.540 | 9.45 | 45.1 | 0.178 |
| NavfnAStar | 157.2 | 58.67 | 0.559 | 9.52 | 76.7 | 0.139 |
| Smac2D | 153.0 | 58.60 | 0.554 | 9.46 | 25.8 | **0.121** |
| **ThetaStar** | **146.1** | 57.38 | **0.540** | **9.40** | **24.2** | 0.150 |
| SmacLattice | 155.7 | 59.02 | 0.569 | 9.64 | 35.9 | 0.196 |

Theta* led after one tour, but a single tour is not enough to change the default, so 5 repeats followed.

**Phase 2 — 5 repeats per planner** (resumed 2026-09-17 in a fresh headless session; planner order rotated every repeat; mean ± 95 % CI, t-distribution; `tools/nav/results/planners_drive_20260917_103512.csv`, log `wp8_repeats.log`). The paused session's repeats 1–2 (`planners_drive_20260916_221042.csv`) are kept but not pooled, because the conditions differ between sessions.

| Planner | Tours ok | Time s | Driven m | Energy Wh | Turning rad | Arrival error m |
|---|---|---|---|---|---|---|
| NavfnDijkstra | 5/5 | 152.7 ± 1.9 | 57.69 ± 0.53 | 0.561 ± 0.015 | 45.2 ± 3.8 | 0.167 ± 0.009 |
| NavfnAStar | 5/5 | 155.0 ± 2.4 | 58.05 ± 0.25 | 0.565 ± 0.013 | 73.0 ± 12.8 | 0.139 ± 0.007 |
| Smac2D | 5/5 | 153.3 ± 0.5 | 58.58 ± 0.19 | 0.566 ± 0.010 | 26.1 ± 0.2 | **0.111 ± 0.005** |
| **ThetaStar** | **5/5** | **147.8 ± 0.9** | **57.26 ± 0.23** | **0.551 ± 0.009** | **24.1 ± 2.5** | 0.153 ± 0.006 |
| SmacLattice | 4/5 | 160.9 ± 8.4 | 59.53 ± 0.69 | 0.585 ± 0.018 | 33.5 ± 2.0 | 0.191 ± 0.009 |

**Paired t-test against NavfnDijkstra** (same repeat = same session conditions):

| Planner | Δ time s | p (time) | Δ energy Wh | p (energy) |
|---|---|---|---|---|
| NavfnAStar | +2.2 ± 2.9 | 0.099 | +0.0043 ± 0.0110 | 0.339 |
| Smac2D | +0.5 ± 1.8 | 0.443 | +0.0052 ± 0.0091 | 0.186 |
| **ThetaStar** | **−5.0 ± 1.7** | **0.001** | **−0.0104 ± 0.0103** | **0.049** |
| SmacLattice | +8.3 ± 10.0 | 0.076 | +0.0248 ± 0.0304 | 0.081 |

**The only failure:** SmacLattice, repeat 4, delivery → charger. It planned 7.78 m for a route the other planners plan at about 5.2 m, drove 5.94 m in 48.8 s and was marked FAIL (goal not reached). The lattice's minimum turning radius near the charger dead end is the likely cause, but this was not investigated further. The other four planners completed all 140/140 legs.

### Decision: ThetaStar is the default planner
- It is the only planner that is significantly better than the old default. Each tour is 5.0 s faster (−3.3 %, p = 0.001) and uses 0.010 Wh less energy (−1.9 %, p = 0.049). It also has the least turning (−47 %), the fastest planning (7.9 ms) and the shortest paths (−0.6 % vs the generated shortest path), with 100 % success.
- Trade-off: its arrival error (0.153 m) is larger than Smac2D's (0.111 m). Both are well inside the 0.35 m arrival tolerance, so this does not matter for the mission.
- Why it helps: any-angle paths remove the stair-step corners of grid planners, so MPPI spends less time turning and decelerating. In this maze the energy gain is small because distance dominates drive energy, and all planners drive nearly the same distance. The time gain is the larger effect.
- Changed: `planner` default = `ThetaStar` in `navigation.launch.py` (argument + `params_with_initial_pose`) and `mission.launch.py`; the `GridBased` block in `nav2_params.yaml` is now Theta* as well. Any other planner is still one argument away: `planner:=NavfnDijkstra`.

### Problem met
- The first repeat attempt found the simulation already stopped; the script waited the full 5 minutes for Nav2 and exited. It now fails with "Nav2 navigation never became active" (a clear message, no hang).

### How to reproduce
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch robofetch_nav navigation.launch.py rviz:=false gz_extra:="-s --headless-rendering" &
ros2 run robofetch_core robot_state_node --ros-args -p use_sim_time:=true &
venv/bin/python -u tools/nav/compare_planners.py --repeats 3 --drive-repeats 5    # phase 1 + phase 2, ~1 h
./scripts/stop.sh
```

## Starter config: choose scenario, model and planner from a file (2026-09-17, awaiting approval)

### What was done
- **`config/run.yaml`** (new) is the starter config that `./scripts/run.sh` reads. Keys: `ask`, `scenario`, `model`, `planner`, `view` (gui|headless), `shift_minutes` (0 = full shift), `seed` (-1 = from params), `dashboard`, `build`.
- **`scripts/run_config.py`** (new) loads and validates the file.
  - Valid scenarios come from the scenario folder; valid planners come from `PLANNERS` in `navigation.launch.py`. A new scenario or planner therefore appears automatically.
  - Unknown or missing keys and invalid values stop the run with the list of valid choices.
  - `save` writes chosen values back into the file, keeping comments and column layout, and refuses to write a file that would not load.
- **`scripts/run.sh`** (rewritten around the config):
  - `ask: true` shows the menu (scenario → model → **planner** → view → minutes). The file's values are the defaults, so Enter keeps them. After confirming, it offers to save the choices as the new defaults.
  - `ask: false` or `--yes` starts straight from the file. `--ask` forces the menu. `--config FILE` uses another starter file.
  - `--dry-run` prints the resolved `ros2 launch` command without starting anything. `--list` now also lists the planners.
  - Command-line values (`scenario:= model:= planner:= seed:= shift_s:= --headless --no-build`) override the file for one run. `model:=` (scripted plan) and pass-through `name:=value` still work. It still refuses to start next to a running simulation.
- **Planner visible live and recorded:**
  - `mission.launch.py` passes `planner` to the mission executor (log line + `planner` in the mission summary) and to the dashboard.
  - The dashboard header now reads "scenario … · model … · planner …", and `/api/state` has `run: {model, planner}`.
  - Runs with different planners can therefore be told apart live and in `logs/*_mission_summary.yaml`.

### Tests (real output)
- `--yes --dry-run` → `mission.launch.py scenario:=balanced planner:=ThetaStar seed:=-1 shift_s:=600.0 web:=true headless:=false model:=ns`.
- `planner:=Smac2D model:=ppo shift_s:=600.0 --headless --no-build --dry-run` → overrides applied, `headless:=true`.
- `model:= --dry-run` → `model:= ai:=false plan:=<demo plan>`.
- `planner:=Foo` → `config error: planner: 'Foo' is not one of: NavfnDijkstra, NavfnAStar, Smac2D, ThetaStar, SmacLattice`, exit 1.
- `ask: true` without a terminal → a clear message to use `--yes`, exit 1.
- Menu driven through a pseudo-terminal (answers 3, 2, 5, 2, 7) → `fault_burst`, `ns_symbolic_only`, `SmacLattice`, headless, `shift_s:=420.0`.
- `save` on a copy → only the three values changed, comments stayed aligned. An invalid save was refused and the file was left unchanged.
- `ros2 launch robofetch_bringup mission.launch.py --show-args` → `planner` default `ThetaStar`.
- The dashboard header renders `model <b>ppo</b> · planner <b>Smac2D</b>` from the launch environment. Core tests: 33 passed.
- Not yet done: a full Gazebo launch through the new `run.sh`. The WP8 simulation was still running, and stopping it is manual.

### How to use
```bash
./scripts/run.sh                    # menu, defaults from config/run.yaml
./scripts/run.sh --yes              # start exactly what config/run.yaml says
./scripts/run.sh planner:=Smac2D    # one-off override (no menu)
./scripts/run.sh --dry-run          # show what would be launched
```

---

## WP9 — Evaluation and system improvement (IN PROGRESS, 2026-09-17)

### 1. Fairness problems found and fixed before evaluating
- **PPO trained on only 4 of its 6 scenarios.** `train_ppo.py` gave each of its 4 parallel environments one fixed scenario (`scenarios[rank % 6]`), so `one_hot_section` and `worn_robot` were never trained on.
  - Fix: `FactoryEnv` accepts a list of scenarios and draws one at random at every reset (seeded per env).
  - Verified: 40 resets covered all 6. PPO masked and unmasked were retrained (9.8 min each).
  - The WP6 model is kept as `tools/ai/checkpoints/ppo_policy_wp6.zip` (policy `ppo_wp6`).
- **Seed overlap.** Evaluation used seeds 0–19. PPO draws random 31-bit seeds and NS collection uses 12345+, so neither was affected. Still, a fixed, documented test range is cleaner.
- **New config `mission.evaluation`:**
  - `training_scenarios`: the six the models train on (both training scripts now read it).
  - `seeds: 30`.
  - `test_seed_start: 5000`: never used by any training or model selection; NS round selection uses 1000–1003.
- **Train/test split:** the six scenarios added after training (`aged_battery`, `heavy_load`, `heavy_parts`, `hot_factory`, `section_breakdown`, `small_buffers`) were never seen by either learned model, so they measure **generalisation**.

### 2. Tooling
- **`tools/ai/evaluate.py`** (rewritten):
  - Parallel workers (3240 episodes in 62 s on 10 workers); test seeds from config.
  - Seen/unseen tag per scenario; paired **Wilcoxon signed-rank** test of every policy against `--reference` (ns).
  - 95 % bootstrap CI; `summary_<ts>.md` with pooled, seen, unseen and per-scenario tables.
  - Decision latency is now measured in every episode (`run_episode` records it; the first, model-loading call is excluded).
- **`tools/ai/plot_results.py`:** thesis figures. Score per scenario, with hatched bars and an "unsafe n/30" label for shifts with violations. Throughput vs Wh per unit. Violation share. Gazebo vs simulator.
- **`tools/ai/run_gazebo_batch.py`:**
  - Runs whole shifts through `run.sh --yes --headless`, waits for the mission summary, stops only its own run, and replays the same policy/scenario/seed/shift in the fast simulator.
  - The score comes from the Gazebo logs with the simulator's rules: section CSVs for lost units, mission CSV for below-reserve, robot CSV for temperature and empty battery.
  - Refuses to start next to a running simulation, and saves after every run (`--skip-done` resumes).
- **`FactorySim(finish_last_action=True)`:** the real executor checks the clock only between actions, so it finishes the last one after the shift ends (a 3-min smoke run lasted 220 s). The comparison mode does the same. With it, the smoke run matched Gazebo: score 3.71 vs 3.71, energy 0.587 vs 0.582 Wh, distance 39.6 vs 39.1 m. Training and evaluation keep the exact cut-off.

### 3. First full evaluation → the weakness it exposed
7 policies × 12 scenarios × 30 seeds (`summary_20260917_150043.md`). NS (unbounded, the WP5 model) was the best or tied in most scenarios, with 0 violations. Pooled, however, it was **below its own symbolic layer** (51.1 vs 52.9, p = 0.005). Almost all of that came from one unseen scenario: **heavy_load 28.5 vs 53.2**.

**Diagnosis** (trace, heavy_load seed 5000):
- NS chose `WAIT:60` while carrying cargo, up to 8 times in a row (t = 1849–2329 s), while A, B and C were BLOCKED (1602 / 1056 / 1020 s blocked; symbolic-only 807 / 675 / 522).
- The explanation string itself said "waiting away from the charger only spends energy". The symbolic estimate was right, but the learned correction (+20 on every action, differences of ~7) overruled it.
- In the training scenarios production is half as fast, and waiting to collect bigger batches pays off. The network carried that habit into an overload it had never seen.

**Tried and rejected:** an "unfamiliar input" guard (feature z-scores vs training statistics). Rare binary features (`target_fault`, `is_deliver`) give |z| ≈ 13 in every scenario, including the training ones, so it is no usable signal.

### 4. Improvement: bounded neural correction (trust region)
`NeuroSymbolicPolicy`: the network's correction is centred on the candidates' mean and clipped to ±`mission.neurosymbolic.max_correction` score units. The network can still decide close calls, but it cannot overrule clear symbolic evidence.

**Chosen on training scenarios only, validation seeds 1000–1019** (never on the test seeds or unseen scenarios):

| bound | none | 20 | 10 | 5 | 2 | **1** | 0 (estimate only) |
|---|---|---|---|---|---|---|---|
| mean score (6 seen × 20 seeds) | 57.75 | 57.75 | 58.06 | 58.15 | 58.15 | **58.70** | 57.41 |

→ `max_correction: 1.0` (params.yaml). The live decision service picks it up automatically. New ablations: `ns_unbounded` (the WP5 model) and `ns_estimate_only` (bound 0: rules + symbolic estimate, network off).

### 5. Final fast-simulator results (test seeds 5000–5029, 9 policies × 12 scenarios × 30 = 3240 shifts)
`tools/ai/results/summary_20260917_150349.md`, `episodes_20260917_150349.csv`, figures `fig_*.png`. p = paired Wilcoxon vs ns.

**All 12 scenarios pooled (360 shifts each):**

| policy | score [95 % CI] | delivered | lost | Wh | Wh/unit | shifts with violation | Δ vs ns | p |
|---|---|---|---|---|---|---|---|---|
| **ns (bounded)** | **53.56 [51.6, 55.4]** | 59.7 | 1.96 | 8.41 | 0.150 | **0/360** | — | — |
| ns_unbounded (WP5) | 51.12 [49.1, 53.2] | 57.8 | 2.63 | 8.08 | 0.148 | 0/360 | −2.44 | <0.001 |
| ns_estimate_only | 52.56 [50.6, 54.5] | 59.1 | 2.17 | 8.76 | 0.159 | 0/360 | −1.00 | <0.001 |
| ns_symbolic_only | 52.91 [50.9, 54.9] | 59.4 | 2.11 | 8.71 | 0.158 | 0/360 | −0.65 | 0.008 |
| ns_neural_only | 51.81 [49.3, 54.2] | 60.7 | 0.21 | 8.56 | 0.152 | **57/360** | −1.75 | 0.144 |
| rule | 41.66 [38.5, 44.7] | 51.6 | 7.47 | 4.97 | 0.099 | 0/360 | −11.90 | <0.001 |
| ppo (retrained, 6 scenarios) | 52.35 [50.3, 54.4] | 61.9 | 0.13 | 10.21 | 0.180 | **53/360** | −1.20 | <0.001 |
| ppo_unmasked | 28.19 [25.8, 30.5] | 44.6 | 8.83 | 7.66 | 0.192 | 42/360 | −25.37 | <0.001 |
| ppo_wp6 (4 scenarios) | 53.75 [51.6, 55.8] | 62.1 | 0.35 | 9.95 | 0.175 | **51/360** | +0.20 | 0.175 |

**Seen (180 shifts each) / unseen (180 each), score and shifts with a violation:**

| policy | seen | unseen |
|---|---|---|
| ns | 60.35, 0 | **46.77, 0** |
| ns_unbounded | 59.79 (p = 0.085), 0 | 42.44 (p < 0.001), 0 |
| ns_symbolic_only | 60.16 (p = 0.97), 0 | 45.66 (p < 0.001), 0 |
| ppo | 59.88 (p = 0.31), 4 | 44.83 (p < 0.001), **49** |
| ppo_wp6 | 59.23, 11 | 48.28 (p = 0.28), **40** |
| ns_neural_only | 52.77, 27 | 50.84, 30 |

**Per scenario, ns vs the strongest alternatives** (Δ = other − ns):
- **heavy_load (unseen):** ns 52.3 (unbounded 28.5, Δ −23.8). symbolic 53.2 (+0.9, p = 0.70). ppo 59.5, but 29/30 shifts unsafe. rule −17.5.
- **Significantly better than symbolic-only:** balanced (+0.93, p = 0.004), heavy_parts (+1.79, p = 0.003), small_buffers (+3.40, p = 0.023).
- **Worse than symbolic-only:** low_battery_start (−0.94, p = 0.045).
- **worn_robot:** the rule policy is still slightly better (+0.93, p = 0.033); the bound fixed most of it (unbounded −2.37 → bounded −0.93).
- **aged_battery:** unbounded is slightly better (+0.82, p = 0.010); ppo 34.3 with 20/30 unsafe.
- **hot_factory:** bounded +2.57 over unbounded (p < 0.001).

### 6. Findings for the thesis
1. **Only the rule-based policies are safe by construction.** NS and symbolic-only had 0 violations in 1440 shifts combined. Without the symbolic layer, the same network (ns_neural_only) is unsafe in 15.8 % of shifts. Masked PPO is unsafe in 14.7 %, mostly in unseen scenarios (49 of 53).
2. **The neural part helps only when it is bounded.** Unbounded, it is worse than the rules alone on unseen scenarios (−3.2). With the ±1 bound (chosen on training data), NS is the best safe policy overall and on unseen scenarios. The neural part adds +1.0 over the bare symbolic estimate (p < 0.001).
3. **PPO's raw score matches NS** on seen scenarios (p = 0.31). It gets there by driving more (10.2 Wh vs 8.4 Wh, 0.180 vs 0.150 Wh/unit) and by accepting low batteries (mean minimum 45 % vs 59 %). Its apparent lead on heavy_load comes with a violation in almost every shift.
4. **Training on all six scenarios did not make PPO better.** ppo_wp6 (4 scenarios) vs ppo (6): no significant difference, both around 15 % unsafe. The violations are a property of learning without rules, not of the missing scenarios.
5. **Without its action mask, PPO does not learn the rules in 300k steps** (28.2, 11.7 % unsafe).
6. **The rule policy is the most energy-efficient** (0.099 Wh/unit, charging 7 times per shift). It collapses when time is short: low_battery_start −7.0, heavy_load −17.5.

### Tests
- `pytest src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test` → **161 passed** (after sourcing the workspace).

### Tier 2 — Gazebo (running)
`run_gazebo_batch.py --models ns ppo --scenarios balanced low_battery_start heavy_load --seeds 5000 5001 --shift-min 20` → `tools/ai/results/gazebo_20260917_150533.csv`, log `wp9_gazebo_batch.log`. Results to follow.

### Tier 2 — Gazebo (done 2026-09-20; paused overnight on user request and resumed)
12 shifts of 20 simulated minutes, headless, planner ThetaStar, `run_gazebo_batch.py`; each replayed in the fast simulator with the same policy, scenario, seed and shift length (`finish_last_action=True`). Results `tools/ai/results/gazebo_20260917_150533.csv`, logs `wp9_gazebo_batch.log` + `wp9_gazebo_batch2.log`, figure `fig_gazebo_vs_sim.png`.

| model | scenario | seed | Gazebo score | sim score | Gazebo Wh | sim Wh | failed actions | shift s |
|---|---|---|---|---|---|---|---|---|
| ns | balanced | 5000 | 16.39 | 17.37 | 3.214 | 3.262 | 0 | 1209 |
| ns | balanced | 5001 | 15.35 | 16.37 | 3.300 | 3.257 | 1 | 1229 |
| ns | heavy_load | 5000 | 32.21 | 33.23 | 3.574 | 3.547 | 0 | 1229 |
| ns | heavy_load | 5001 | 32.28 | 33.23 | 3.438 | 3.546 | 0 | 1202 |
| ns | low_battery_start | 5000 | **8.61** | 15.91 | 0.776 | 3.025 | 1 | **266 (halted)** |
| ns | low_battery_start | 5001 | 17.69 | 18.87 | 2.956 | 2.913 | 0 | 2070 |
| ppo | balanced | 5000 | 13.09 | 17.16 | 3.822 | 3.672 | 0 | 1248 |
| ppo | balanced | 5001 | 18.17 | 18.12 | 3.652 | 3.751 | 0 | 1225 |
| ppo | heavy_load | 5000 | 30.03 | 31.09 | 3.948 | 3.829 | 0 | 1245 |
| ppo | heavy_load | 5001 | 28.66 | 32.07 | 3.879 | 3.859 | 1 | 1213 |
| ppo | low_battery_start | 5000 | 19.03 | 19.01 | 1.941 | 1.977 | 0 | 1223 |
| ppo | low_battery_start | 5001 | 19.03 | 20.01 | 1.932 | 1.982 | 0 | 1216 |

**Over the 11 shifts that ran to the end:**

| metric | Gazebo | fast sim | gap |
|---|---|---|---|
| score | 21.99 | 23.32 | −5.7 % |
| delivered units | 23.73 | 25.00 | −5.1 % |
| energy Wh | 3.24 | 3.24 | **+0.2 %** |
| safety violations | 0 | 0 | — |

Per shift the score gap is −1.33 on average (worst −4.07, best +0.05). **Energy transfers almost exactly**; the small score gap is throughput: real driving loses a few seconds per leg (Nav2 recovery, acceleration, arrival tolerance), which costs about one delivered unit per 20-minute shift. The ranking is unchanged: ns ≥ ppo on `balanced` and `heavy_load` in Gazebo too, and no policy had a violation in a completed shift.

The 20-minute shifts are too short for the low-battery differences of tier 1 to appear: both models simply work until the battery forces one charge. `ns / low_battery_start / 5001` needed 2070 s because the last action was a charge (the executor finishes a started action).

### Navigation problems found in the Gazebo runs (not decision-layer problems)
Three of 12 shifts hit a navigation fault. In each case the recovery ladder did its job; the decision models were never at fault.
1. **`ns / balanced / 5001`, action 18: Theta\* refused to plan to C.** Three aborts, `GridBased plugin failed to plan from (4.18, -3.99) to (6.03, -3.42): "Either of the start or goal pose are an obstacle!"`. The robot gave up on C, returned to the charger (84 s, 0.24 Wh) and finished the shift. Theta\* has no goal tolerance; NavFn plans to the nearest free cell within 0.5 m instead. A momentarily occupied goal cell near machine C is therefore fatal for Theta\* only. WP8's 35 test legs never hit it.
2. **`ns / low_battery_start / 5000`: the planner server stalled and the robot halted after 266 s.** B unreachable (3 attempts), then the charger unreachable as well → `navigation_stuck`, as designed. The cause is in Nav2, not in the maze: `Planner loop missed its desired rate of 20 Hz. Current loop rate is 1.87 Hz` and `bt_navigator: Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose` (5×). The planner server was not answering in time.
3. **`ppo / heavy_load / 5001`: one failed action**, same pattern, shift continued.

### Planner check (2026-09-20): the default goes back to NavfnDijkstra
The same shift (`ns / low_battery_start / seed 5000`, 20 min) was driven **5 times per planner, alternating** ThetaStar and NavfnDijkstra, so machine load hits both equally (`run_gazebo_batch.py --planner ...`, new flag; logs `plannercheck_<planner>_r<n>.log`, rows `plannercheck_<planner>_r<n>.csv`).

| planner | planner-loop stalls | plan refusals | nav retries | clean shifts | score (n = 5) | delivered | Wh |
|---|---|---|---|---|---|---|---|
| ThetaStar | 6 (worst 1.11 Hz) | 3 | 6 | **2/5** | 18.07 ± 1.61 sd | 20.2 | 2.685 |
| **NavfnDijkstra** | 0 | 0 | 0 | **5/5** | 17.75 ± 1.23 sd | 20.6 | 2.927 |

- Every fault is on the ThetaStar side, in both directions of the alternation, so it is **not** CPU starvation.
- The mechanism is in the message: `GridBased plugin failed to plan from (-5.54, 1.43) to (-5.97, 3.08): "Either of the start or goal pose are an obstacle!"`. Here the **start** was the problem: the robot's own pose next to section B was momentarily inside the inflation layer. Theta\* has no goal/start tolerance and refuses outright; NavFn plans from the nearest free cell within `tolerance: 0.5`. The 20 Hz → 1.1 Hz planner-loop stalls only ever appear together with those refusals, so they are a symptom, not a separate fault.
- Over whole shifts ThetaStar's WP8 advantage disappears: 18.07 vs 17.75 with a run-to-run sd of ~1.4, i.e. no difference. WP8's ~3 % gain was measured on 7-leg tours where recovery and dwell time play no part.
- **Across all Gazebo shifts: ThetaStar faulty in 6 of 17, NavfnDijkstra in 0 of 5.**

**Changed:** `planner` default back to `NavfnDijkstra` in `navigation.launch.py` (argument + `params_with_initial_pose`), `mission.launch.py`, `config/run.yaml`, and the `GridBased` block of `nav2_params.yaml`; `run.sh`'s planner menu says which is reliable and which is fastest. ThetaStar is still one argument away (`planner:=ThetaStar`) and its WP8 numbers stand.

**Thesis point:** a planner chosen on short benchmark tours (WP8) was the *worse* choice in operation. Only full-shift runs exposed it, because the failure needs a pose that is momentarily inside the inflation layer - a situation short, clean tours never produce.

---

## WP9 follow-up (2026-09-20): H5 rule, maze on the dashboard, remaining gaps explained

### 1. H5 "pointless waiting" (symbolic layer)
WP9 saw the robot waiting at a section, ~1 % of the pack at a time, before driving to the charger anyway. The cause was not the ±1 bound but a **gap between two thresholds**: pickups are refused as soon as the return trip would break the reserve, while charging only becomes urgent (tier 2) slightly *below* that battery level. In between, waiting and charging looked equally good.

New hard rule in `SymbolicLayer.evaluate` (it needs the whole action set, so it is applied after the per-action checks): **away from the charger, with every PICKUP and DELIVER already forbidden and charging possible, WAIT is forbidden** — nothing can change except the battery draining.

- Validation (training scenarios, seeds 1000–1019): 58.70 → **58.81**; `low_battery_start` 53.44 → 54.10; no scenario got worse.
- Tests: two new ones in `test_symbolic.py` (the rule fires when only charging is left; waiting stays allowed while the robot can still work). **163 passed.**
- Full test run (`summary_20260920_210847.md`, same 3240 shifts): pooled ns **53.56 → 53.81**, unseen **46.77 → 47.20**, still **0/360** violations. `low_battery_start`: ns 55.14 → 55.57, so symbolic-only's lead is no longer significant (+0.51, p = 0.100, was p = 0.045).

### 2. The maze on the dashboard
`robofetch_bridge/maze.py` (`MazeView`) draws the hall from the **same `layout.yaml`** the Gazebo world and the Nav2 map are generated from, so the picture cannot drift from what the robot drives in. Cells of a row are merged into runs (~40 rectangles instead of 580), section pads keep their layout colours, and the SVG is in metres, so it scales with the card. `ros_link` keeps a trail (one point per ~0.15 m, 400 points ≈ 60 m of route) and the page draws the robot on top. Dark mode is handled with CSS variables. Verified by rendering the page with a mock snapshot: map card, SVG and robot marker all present; geometry checked against `poi.yaml` (charger at world (−5.98, −4.0) → picture (1.27, 9.0), bottom-left, as in the world).

### 3. The last two gaps, explained
- **`low_battery_start`** — closed by H5 (see above).
- **`worn_robot`** (rule 49.17 vs ns 48.24, p = 0.033): not a fault but a **batching trade-off**. Over 10 seeds: the rule policy makes **14 deliveries of 3.71 units** and drives **359 m**; NS makes **25 deliveries of 2.24 units** and drives **468 m**, for the same ~52 units delivered (battery energy 5.45 vs 7.77 Wh). The rule waits *on the charger*, where idling is free and the pack tops up, until buffers are worth a trip; NS reacts sooner, which wins in every other scenario but costs distance exactly where a worn drivetrain makes each metre expensive.
  Candidates, none applied (each would be tuned on training scenarios first): a longer rollout horizon in NS training so batching is valued, or a soft rule "do not deliver a part-load while no section is near full". Deliberately not hacked for one scenario.
- **Noted while investigating:** the objective charges only the energy *drawn from the battery*. Idling docked is paid by the charger, so a policy that parks on the charger looks cheaper than one that waits in the hall. Physically the factory pays for both. Changing it would mean re-training both models, so it is left as it is and stated here as a limitation.

### 4. 60-minute Gazebo shifts: the safety difference, in the full simulation
20-minute shifts are too short for the battery to become critical, so one full 60-minute shift per model was driven on `low_battery_start`, seed 5000 (`run_gazebo_batch.py --shift-min 60`, `tools/ai/results/gazebo_60min.csv`, log `wp9_longshift.log`), with NavfnDijkstra and the H5 rule.

| model | Gazebo score | sim score | delivered | Wh | min battery | violations | failed actions |
|---|---|---|---|---|---|---|---|
| **ns** | **55.16** | 53.38 | 61 | 7.356 | **17.0 %** | **0** | 0 |
| ppo | 37.79 | 57.20 | 62 | 8.421 | **9.4 %** | **1** | 0 |

- **PPO ran the battery below the 15 % reserve and kept working**: after action 42 (DELIVER) 14.9 %, then PICKUP:B 14.2 %, DELIVER 13.5 %, PICKUP:A 12.9 %, bottoming out at 9.4 %. In a real hall that is a robot risking a strand in a corridor.
- **NS never went below 17.0 %**: H3 refuses any trip it could not return from with the reserve, and H5 sends it to the charger instead of waiting. Same scenario and seed, almost the same output (61 vs 62 units), less energy, no violation. The score gap is that violation's penalty.
- This is the tier-1 finding (PPO unsafe in 14.7 % of shifts) reproduced with real navigation and timing.
- PPO's fast-simulator replay of the same seed happened to stay above the reserve (57.20, no violation): its policy runs close to the limit, and small timing differences decide on which side it lands. NS has the same kind of timing spread (55.16 Gazebo vs 53.38 sim) but a hard margin, so it lands on the safe side either way.

### Open issues
- The objective's energy accounting (above).
- `worn_robot` batching (above).
- One 60-minute shift per model is a demonstration, not a statistic; more seeds would turn it into one (~65 min wall per shift).

