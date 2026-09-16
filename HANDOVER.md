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
