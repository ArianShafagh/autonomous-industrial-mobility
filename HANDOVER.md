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
