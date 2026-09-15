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
