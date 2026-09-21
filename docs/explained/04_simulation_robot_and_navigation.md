# 04 — Simulation, robot model (URDF) and navigation (Nav2)

This section covers the **motion layer**: the simulated robot, the physics, and how Nav2 turns "go to B" into wheel commands. It also covers the unfinished WP8 planner comparison.

Files:
- `src/robofetch_description/urdf/robofetch.urdf.xacro`, `robofetch.gazebo.xacro`, `launch/rsp.launch.py`
- `src/robofetch_gazebo/launch/sim.launch.py`, `config/bridge.yaml`, `worlds/factory_maze.sdf`
- `src/robofetch_nav/config/nav2_params.yaml`, `launch/navigation.launch.py`, `maps/`
- `tools/nav/compare_planners.py` (uncommitted), `scripts/check_nav.py`

---

## 1. The robot description (URDF + xacro)

**URDF** (Unified Robot Description Format) is XML describing *links* (rigid bodies with visual, collision and inertia) and *joints* (how links move relative to each other). **xacro** adds variables and macros so values are not repeated.

### 1.1 The robot, part by part

| Link | Shape | Mass | Joint to parent |
|---|---|---|---|
| `base_footprint` | none (ground frame) | — | root |
| `base_link` | box 0.35 × 0.22 × 0.10 m, lifted 0.03 m | 3.0 kg | fixed, z = +0.06 (wheel radius) |
| `left_wheel`, `right_wheel` | cylinder r = 0.06, w = 0.04 | 0.5 kg each | **continuous** (unlimited rotation), axis y, at y = ±0.13 |
| `front_caster`, `rear_caster` | sphere r = 0.03 | 0.1 kg each | fixed, x = ±0.14, z = −0.03 |
| `lidar_link` | cylinder r = 0.04 | 0.1 kg | fixed, x = 0.10, z = 0.07 |

Total mass = 3.0 + 2×0.5 + 2×0.1 + 0.1 = **4.3 kg**, which is exactly `robot.mass_kg` in `params.yaml`. The energy model's "payload energy per kg" is derived from this mass (file 05).

### 1.2 Inertia macros

Physics needs each link's inertia tensor. The macros implement the textbook formulas:

- Box: $I_{xx} = \frac{m}{12}(y^2+z^2)$, $I_{yy} = \frac{m}{12}(x^2+z^2)$, $I_{zz} = \frac{m}{12}(x^2+y^2)$
- Cylinder (axis z): $I_{xx}=I_{yy}=\frac{m}{12}(3r^2+l^2)$, $I_{zz}=\frac{m}{2}r^2$
- Sphere: $I = \frac{2}{5}mr^2$

Wrong inertias make simulated robots wobble, flip or slide. Deriving them from the dimensions avoids that.

### 1.3 Design comments that encode real lessons

- **Wheels directly under the centre of mass** (`wheel_xoff = 0`). Offsetting them shifts weight onto the casters. The drive wheels then lose traction, skid in turns and **corrupt odometry**, and AMCL depends on odometry.
- **Casters with near-zero friction (μ = 0.02, not 0).** Perfectly frictionless contacts are numerically degenerate and jitter.
- **Wheels μ = 1.5, high contact stiffness** (`kp = 1e6`, `kd = 100`) so they grip.
- **Chassis lifted 3 cm** so it does not scrape the floor.

### 1.4 Gazebo plugins (`robofetch.gazebo.xacro`)

| Plugin / sensor | What it does | Settings |
|---|---|---|
| `gz-sim-diff-drive-system` | Subscribes `cmd_vel` (Twist), turns the wheel joints, publishes `odom` and TF `odom → base_footprint` | separation 0.26 m, radius 0.06 m, 30 Hz, max linear accel 1.0 m/s² |
| `gz-sim-joint-state-publisher-system` | Publishes wheel angles → `robot_state_publisher` rotates the wheel links | — |
| `gpu_lidar` sensor on `lidar_link` | 2D laser scan | 360 samples over 360°, 0.12–12 m, 1 cm resolution, 10 Hz |
| `gz-sim-pose-publisher-system` | **Ground-truth** model pose on `/model/robofetch/pose` | 5 Hz, model pose only |

**Concept: differential drive kinematics.** With wheel radius $r = 0.06$ m, separation $L = 0.26$ m and wheel angular speeds $\omega_R, \omega_L$:

$$v = \frac{r(\omega_R + \omega_L)}{2}, \qquad \omega = \frac{r(\omega_R - \omega_L)}{L}$$

The plugin inverts this: given the commanded $(v, \omega)$ from `/cmd_vel`, it sets the wheel speeds. A diff-drive robot cannot move sideways, which is why Nav2's MPPI uses `motion_model: DiffDrive` and `min_y_velocity_threshold: 0.5` (y velocity is effectively ignored).

**Why ground truth?** A real robot has no perfect pose. The project uses ground truth **only to measure**: true distance driven (`mission_executor._on_truth`), localisation error (`check_nav.py`), and goal error (`compare_planners.py`). Arrival verification deliberately uses AMCL, not ground truth, because *"a real robot has none"* (`_drive_once` comment).

---

## 2. Launching the simulation (`sim.launch.py`)

1. `gz_sim.launch.py` with `factory_maze.sdf -r -v4` plus `gz_extra` (`-s --headless-rendering` for headless: server only, offscreen rendering so the GPU lidar still works).
2. `rsp.launch.py` runs `xacro` → URDF string → `robot_state_publisher`, which publishes `/robot_description` and fixed-joint TF.
3. `ros_gz_sim create -topic /robot_description -name robofetch -x … -y … -z 0.1 -Y …` spawns the robot at the **charger pose read from `poi.yaml`**.
4. `ros_gz_bridge parameter_bridge` runs with `bridge.yaml`.
5. Optional RViz.

`QT_QPA_PLATFORM=xcb` forces X11, because on WSLg Gazebo's Qt Quick window otherwise never appears.

### 2.1 The bridge (`bridge.yaml`)

Gazebo has its own transport (gz-transport) with its own message types. The bridge converts:

| ROS topic | Direction | ROS type ↔ gz type |
|---|---|---|
| `cmd_vel` | ROS → GZ | `geometry_msgs/Twist` ↔ `gz.msgs.Twist` |
| `odom` | GZ → ROS | `nav_msgs/Odometry` ↔ `gz.msgs.Odometry` |
| `tf` | GZ → ROS | `tf2_msgs/TFMessage` ↔ `gz.msgs.Pose_V` |
| `joint_states` | GZ → ROS | `sensor_msgs/JointState` ↔ `gz.msgs.Model` |
| `scan` | GZ → ROS | `sensor_msgs/LaserScan` ↔ `gz.msgs.LaserScan` |
| `clock` | GZ → ROS | `rosgraph_msgs/Clock` ↔ `gz.msgs.Clock` |
| `/model/robofetch/pose` | GZ → ROS | `geometry_msgs/Pose` ↔ `gz.msgs.Pose` |

The comment on the last entry records a gotcha: the bridge **drops entity names** when converting the bulk `Pose_V` topic, so each model needs its own named pose topic.

---

## 3. Nav2 overview

**Nav2** is ROS 2's navigation framework. The pipeline for one `NavigateToPose` goal:

```
            ┌───────────── bt_navigator (Behaviour Tree) ─────────────┐
goal pose ─►│ ComputePathToPose ──► FollowPath ──► (goal reached?)    │
            │        │ replan 1 Hz     │                              │
            │        ▼ on failure: ClearCostmap, Spin, Wait, BackUp   │
            └────────┼─────────────────┼──────────────────────────────┘
                     ▼                 ▼
             planner_server      controller_server (MPPI, 10 Hz)
             (global costmap)    (local costmap)
                                       │ cmd_vel
                                       ▼
                     velocity_smoother ─► collision_monitor ─► /cmd_vel ─► Gazebo
        map_server ─► /map        amcl: /scan + /odom + /map ─► map→odom TF
```

---

## 4. Localisation: AMCL

### 4.1 Concept: particle filter (Monte Carlo Localisation)

The robot's pose is uncertain. AMCL represents the belief as **N particles**, each a guessed pose (x, y, θ) with a weight. Each cycle:

1. **Predict (motion model):** move every particle by the odometry change plus random noise. The noise grows with how much the robot moved.
2. **Update (sensor model):** for each particle, compute how well the lidar scan would match the map *if* the robot were at that particle. The weight is proportional to that likelihood.
3. **Resample:** draw a new particle set in proportion to the weights. Good hypotheses multiply and bad ones die.
4. **Estimate:** publish the weighted mean pose on `/amcl_pose` and the `map → odom` transform.

"Adaptive" (the A in AMCL) means the particle count adapts through **KLD sampling**: many particles when uncertain, few when confident. The bounds here are `min_particles: 500`, `max_particles: 2000`, with `pf_err: 0.05` and `pf_z: 0.99` as the KLD bound parameters.

### 4.2 This project's AMCL settings (and why)

| Parameter | Value | Meaning / reason |
|---|---|---|
| `robot_model_type` | `DifferentialMotionModel` | diff-drive noise model |
| `alpha1..alpha4` | **0.05** (Nav2 default 0.2) | Odometry noise: α1 rotation from rotation, α2 rotation from translation, α3 translation from translation, α4 translation from rotation. Low because simulated odometry is accurate to ~0.5 % (comment in the file) |
| `laser_model_type` | `likelihood_field` | Pre-computes, for each map cell, the distance to the nearest obstacle. A beam endpoint's likelihood is a Gaussian of that distance (`sigma_hit: 0.2`) mixed with random noise (`z_hit 0.5`, `z_rand 0.5`). Fast and smooth |
| `max_beams` | 120 | uses 120 of the 360 beams per update (speed) |
| `laser_likelihood_max_dist` | 2.0 | obstacle-distance lookup horizon |
| `update_min_d`, `update_min_a` | 0.10 m, 0.10 rad | filter update every 10 cm / ~6° (default 0.25) so the estimate does not lag |
| `recovery_alpha_fast/slow` | 0.1 / 0.001 | **Augmented MCL**: tracks short- and long-term average likelihood. If the fast average drops below the slow one (the scan suddenly matches poorly), random particles are injected, so the filter can recover from being confidently wrong (kidnapped robot) |
| `set_initial_pose` + `initial_pose` | injected at launch | see §4.3 |
| `transform_tolerance` | 1.0 s | how old the `map→odom` TF may be |

Measured quality (HANDOVER WP1 / WP3): maximum AMCL error against ground truth was **0.071 m** on the WP1 tour and **0.077 m** after the 4 ms physics change.

### 4.3 The initial-pose race condition (WP3, important)

Nav2's planner and controller **cannot activate** until the `map → odom` transform exists. AMCL publishes it **only after it gets an initial pose**. If nobody sends one within the activation timeout (~60 s), bringup aborts. Early runs worked only because a pose happened to arrive in time.

The fix, in `navigation.launch.py → params_with_initial_pose()`:

```python
spawn = poi["poi"][poi["spawn"]]                        # the charger pose, generated
amcl["set_initial_pose"] = True
amcl["initial_pose"] = {"x": spawn["x"], "y": spawn["y"], "z": 0.0, "yaw": spawn["yaw"]}
planners["GridBased"] = dict(planners[planner])         # WP8: choose the planner
out = tempfile.NamedTemporaryFile(...); yaml.safe_dump(params, out)   # a temp params file
```

This runs inside an `OpaqueFunction` because it needs the *resolved* `params_file` and `planner` arguments. The executor still publishes `/initialpose` until AMCL answers (a harmless double safety).

---

## 5. Costmaps

### 5.1 Concept

A **costmap** is a grid where each cell has a cost from 0 (free) to 254 (lethal), plus 255 for unknown. It is built from **layers**:

| Layer | Global costmap | Local costmap |
|---|---|---|
| `static_layer` (the map) | ✔ | — (plugin defined but not in the plugin list) |
| `obstacle_layer` (2D lidar marking/clearing) | ✔ | — |
| `voxel_layer` (3D voxel grid from lidar) | — | ✔ |
| `inflation_layer` | ✔ | ✔ |

- **Global costmap:** frame `map`, whole map, 1 Hz, used by the **planner**.
- **Local costmap:** frame `odom`, **rolling 3 × 3 m window** around the robot, 5 Hz, used by the **controller**.
- Both use 0.05 m resolution and `robot_radius: 0.22` (a circle around the 0.35 × 0.22 m box, since the circumscribed radius is √(0.175² + 0.11²) = 0.207 m).

**Marking and clearing:** a lidar hit *marks* the cell as an obstacle (within `obstacle_max_range: 2.5` m). Cells along the beam are *cleared* by ray tracing (`raytrace_max_range: 3.0` m). This is how a box added with `obstacle.sh` appears, and how it disappears after removal.

### 5.2 Inflation, with the formula and a worked example

Cells near obstacles get a decaying cost so paths keep a distance:

$$\text{cost}(d) = \begin{cases} 254 & d = 0 \text{ (lethal)} \\ 253 & d \le r_{inscribed} \\ 252 \cdot e^{-k\,(d - r_{inscribed})} & r_{inscribed} < d \le r_{inflation} \\ 0 & d > r_{inflation} \end{cases}$$

With $r_{inscribed} = 0.22$, $k$ = `cost_scaling_factor` = 3.0 and $r_{inflation}$ = 0.35:

| Distance from wall | Cost |
|---|---|
| 0.20 m | 253 (robot centre here = collision) |
| 0.25 m | 252·e^(−0.09) ≈ 230 |
| 0.30 m | 252·e^(−0.24) ≈ 198 |
| 0.35 m | 252·e^(−0.39) ≈ 171 |
| 0.36 m | 0 |

This explains the WP1 pinch bug (file 03 §8): a planner only forbids d ≤ 0.22 m, so a 0.5 m gap is plannable even though its cost is high everywhere.

**Stale comment:** the header of `nav2_params.yaml` says `inflation_radius: 0.70 -> 0.45`, but both costmaps are actually set to **0.35**.

---

## 6. Global planners (and WP8)

The planner computes a path on the **global costmap** from the robot to the goal. The behaviour tree uses the plugin named `GridBased`. `navigation.launch.py` copies the chosen algorithm's settings into that name (`planner:=ThetaStar`). All five are also loaded side by side so a benchmark can call each by `planner_id`.

| Planner | Algorithm | Key settings here |
|---|---|---|
| **NavfnDijkstra** (default) | Wavefront Dijkstra from the goal over the costmap, gradient descent to extract the path | `tolerance 0.5` (plan to the nearest free cell within 0.5 m if the goal is blocked) |
| **NavfnAStar** | Same cost function, A* heuristic search | `use_astar: true` |
| **Smac2D** | Cost-aware 8-connected A* + path smoother | `cost_travel_multiplier 2.0`, `tolerance 0.25`, smoother `w_smooth 0.3`, `w_data 0.2` |
| **ThetaStar** | **Any-angle** A*: a node can take a grandparent as parent if line of sight exists, so paths are straight lines between visible corners | `w_euc_cost 1.0`, `w_traversal_cost 2.0`, `how_many_corners 8` |
| **SmacLattice** | State-lattice search over **kinematically feasible motion primitives** for a diff-drive (5 cm, 0.5 m turning radius) | penalties for reverse, direction change, non-straight, cost, rotation |

**Concept: A\* vs Dijkstra.** Dijkstra expands nodes by cost-so-far g(n). A* expands by f(n) = g(n) + h(n), where h is an admissible estimate of the remaining cost (e.g. Euclidean distance to the goal). A* finds the same optimal path while exploring fewer nodes.

**Concept: any-angle planning.** Grid planners restrict headings to 45° steps, so paths zig-zag and are ~8 % longer than the true shortest path. Theta* removes that restriction, which is exactly what the path matrix's string pulling approximates (file 03 §7.3).

**Concept: state lattice.** Instead of grid cells, the search graph's edges are short, pre-computed, drivable curves. Paths are feasible for the robot's kinematics without post-processing, at a higher computation cost.

### 6.1 The goal-tolerance side effect (found in WP3b)

`tolerance: 0.5` means that **if the goal is blocked, Nav2 plans to the nearest free spot and reports SUCCESS there**. The first obstacle test showed PICKUP:C "succeeding" with a box sitting on C. The executor therefore verifies arrival itself (file 07).

### 6.2 WP8 results so far (from `HANDOVER.md`)

The method is in `tools/nav/compare_planners.py`. It **isolates the planner**: each planner computes the path (`ComputePathToPose` with `planner_id`), and the **same MPPI controller** follows it (`FollowPath`). Any difference in time or energy is caused by the path.

**Phase 1** covers planning only, 20 ordered POI pairs × 3 repeats, 300/300 planned:

| Planner | Plan ms mean / p95 | Length vs matrix | Total turning (rad) | Min clearance (m) |
|---|---|---|---|---|
| NavfnDijkstra | 15.3 / 36.0 | +1.0 % | 7.40 | 0.34 |
| NavfnAStar | 18.6 / 35.6 | +2.3 % | 11.59 | 0.32 |
| Smac2D | 9.2 / 20.0 | +0.7 % | 4.88 | 0.28 |
| **ThetaStar** | **7.9 / 18.2** | **−0.6 %** | **4.07** | 0.32 |
| SmacLattice | 129.6 / 680.0 | +6.8 % | 5.21 | 0.29 |

**Phase 2** is one driven tour per planner, 35/35 legs:

| Planner | Time (s) | Driven (m) | Energy (Wh) | Wh/km | Turning (rad) | Arrival error (m) |
|---|---|---|---|---|---|---|
| NavfnDijkstra | 151.5 | 57.17 | 0.540 | 9.45 | 45.1 | 0.178 |
| NavfnAStar | 157.2 | 58.67 | 0.559 | 9.52 | 76.7 | 0.139 |
| Smac2D | 153.0 | 58.60 | 0.554 | 9.46 | 25.8 | 0.121 |
| **ThetaStar** | **146.1** | 57.38 | **0.540** | **9.40** | **24.2** | 0.150 |
| SmacLattice | 155.7 | 59.02 | 0.569 | 9.64 | 35.9 | 0.196 |

**How to read this honestly:**
- ThetaStar leads on time (3.6 % faster than the default), turning and planning speed. It is a **single tour** though, and repeats were stopped during repeat 3, so there is **no statistical conclusion yet**.
- Energy differences are tiny (0.540 vs 0.569 Wh, about 5 %). The reason is structural (file 05): at 0.39 m/s, **idle electronics dominate energy**, so a planner mostly saves energy by saving *time*.
- "Length vs matrix −0.6 %" for Theta* means Theta* found slightly shorter paths than the generator's string-pulled estimate, which confirms the matrix is a slightly conservative approximation.
- Phase-1 "turning" counts heading changes along the planned path, while phase-2 "turning" is summed over driven legs (different sums, not comparable across the two tables).

**Statistical design in the tool (good practice):**
- `--drive-repeats N` **rotates planner order** each repeat, so slow drift (battery level, motor temperature, simulator load) cannot favour whichever planner goes first. This is counterbalancing.
- `report_repeats` prints the mean ± 95 % CI (Student t) and a **paired t-test** against NavfnDijkstra (the same repeat index = the same session conditions).
- Results are saved after every repeat.

**Bug found while reading (not yet triggered in the logs):** `PlannerBench.follow()` returns a **2-tuple** `(False, "rejected")` when the FollowPath goal is rejected, but `phase_drive` unpacks **three** values (`ok, detail, driven = bench.follow(path)`). A rejected goal would crash the benchmark with `ValueError` instead of recording a failed leg. Fix: `return False, "rejected", 0.0`.

---

## 7. Controller: MPPI

### 7.1 Concept: Model Predictive Path Integral control

MPPI is a **sampling-based model predictive controller**. Every control cycle (10 Hz here):

1. Start from the previous optimal control sequence $U = (u_0, …, u_{T-1})$, with $T$ = `time_steps` = 32 steps × `model_dt` = 0.1 s, so the horizon is **3.2 s**.
2. Sample $K$ = `batch_size` = **600** noisy versions: $U_k = U + \epsilon_k$, with noise std `vx_std 0.2`, `wz_std 0.4`.
3. **Roll out** each sequence through the DiffDrive motion model to get 600 predicted trajectories.
4. Score each trajectory with the **critics** (weighted costs), giving $S_k$.
5. Compute weights by a softmax of negative cost: $w_k = \exp(-S_k/\lambda) / \sum_j \exp(-S_j/\lambda)$, with $\lambda$ = `temperature` = 0.3. Low temperature means only the best trajectories matter.
6. Update $U \leftarrow \sum_k w_k U_k$, execute $u_0$, and shift the sequence.

MPPI handles non-linear dynamics and arbitrary, non-differentiable costs (like costmap lookups), because it only *evaluates* samples and never differentiates.

### 7.2 Critics used

| Critic | Weight | Pushes the trajectory to… |
|---|---|---|
| `ConstraintCritic` | 4.0 | respect velocity/acceleration limits |
| `CostCritic` | 3.81 | stay away from high costmap cost; `collision_cost 1e6` for lethal |
| `GoalCritic` | 5.0 | get close to the goal position (within 1.4 m) |
| `GoalAngleCritic` | 3.0 | face the goal yaw (within 0.5 m) |
| `PathAlignCritic` | **14.0** | stay on the global path (strongest weight, so the controller follows the planner closely, which is why WP8 can isolate the planner) |
| `PathFollowCritic` | 5.0 | make progress along the path |
| `PathAngleCritic` | 2.0 | point along the path |
| `PreferForwardCritic` | 5.0 | drive forward, not backward |

### 7.3 CPU budget comment in the config

The default MPPI batch/time steps starved the control loop on WSL with the Gazebo GUI and RViz running, so the robot stalled mid-corridor. Reduced values (600 × 32 at 10 Hz) run comfortably. Also note the constraint that the controller period must be ≤ `model_dt` (0.1 s ↔ 10 Hz).

### 7.4 Goal and progress checkers

- `SimpleGoalChecker`: `xy_goal_tolerance 0.25`, `yaw_goal_tolerance 0.25`, `stateful: true` (once the position is reached, only rotate). The comment says **leave it at 0.25**: 0.12 and 0.20 were tried, MPPI could not park that precisely, the goal checker never passed, and goals aborted.
- `SimpleProgressChecker`: the robot must move 0.25 m within 30 s or the goal fails. This is Nav2's own "stuck" detector. The executor adds a longer overall timeout (file 07).

(The same comment mentions "snapping the parcel to a carry position". That refers to the old gripper project and is stale.)

### 7.5 Downstream of the controller

- `velocity_smoother` limits acceleration (±2.5 m/s², ±3.2 rad/s²) and velocity (±0.5 m/s, ±2.0 rad/s), open-loop, at 20 Hz.
- `collision_monitor` uses `FootprintApproach`: it projects the footprint 1.2 s ahead with the current velocity and slows down so it would stop before contact with lidar points.

**Speed numbers:** MPPI `vx_max` is 0.5 m/s, but the **measured average in the maze including turns is 0.39 m/s** (WP1: 57.27 m / 147.4 s). That measured value is `robot.motion.speed_m_s` in `params.yaml`, which is what the AI's time predictions use. The header comment "max speeds raised to 0.40 m/s" is stale.

---

## 8. Behaviour tree navigator and behaviours

`bt_navigator` runs Nav2's default XML tree `navigate_to_pose_w_replanning_and_recovery.xml`: replan the path at 1 Hz while following it, and on failure run recoveries (clear costmaps, spin, wait, back up) before giving up. Behaviours available in `behavior_server` are `spin`, `backup`, `drive_on_heading`, `assisted_teleop`, `wait`. The executor calls `backup` directly as step 2 of its own recovery ladder.

Not used by the project but configured because `nav2_bringup` starts them anyway: `waypoint_follower`, `route_server` (points to a sample graph so it configures), `docking_server`, `smoother_server`.

---

## 9. Measured navigation quality (from HANDOVER)

WP1 tour (charger → A → delivery → B → delivery → C → delivery → charger): **7/7 legs**, 57.27 m, 147.4 s sim, average 0.389 m/s, max goal error 0.217 m (inside the 0.25 m tolerance), max AMCL error 0.071 m.

WP3b obstacle tests:
- A box on C's pickup pose: Nav2 "succeeded" 0.62 m away. The executor detected it, cleared costmaps, retried, and succeeded.
- A box on the charger: timeout, then abort, then timeout. The full ladder ran, the robot halted, and the mission ended as `navigation_stuck`.

---

## 10. Key concepts of this section

- URDF links/joints/inertia, xacro macros, `base_footprint`/`base_link` convention
- Differential-drive kinematics
- Simulator ↔ middleware bridging (gz-transport ↔ DDS)
- Monte Carlo localisation, KLD adaptive sampling, likelihood-field sensor model, augmented MCL recovery
- Layered costmaps, marking/clearing, exponential inflation
- Dijkstra vs A* vs any-angle (Theta*) vs state-lattice planning
- MPPI sampling-based MPC, critics, temperature
- Behaviour trees for navigation logic, goal/progress checkers
- Experimental isolation (same controller, only the planner varies), counterbalancing, paired tests

---

## 11. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Finish WP8 with enough repeats** | One tour cannot separate ThetaStar from the default. The paired design is already there | Run `compare_planners.py --skip-plan --drive-repeats 10`; report paired CI and p-values; also fix the `follow()` 2-tuple bug first |
| **Evaluate planners by what matters to the AI** | The decision layer uses a *time* model; planners should be judged on time variance, not only mean | Store per-leg time distributions; feed the measured mean speed per planner back into `speed_m_s` |
| **Make the energy model planner-sensitive** | Turning and acceleration are not in `robot_model` (energy = distance × Wh/m), so a jerky path and a smooth one of equal length cost the same | Add an angular term (Wh per rad turned) and an acceleration term to `drive_energy_wh`, calibrated from Gazebo wheel torques (`gz-sim` joint force) |
| **Keepout / speed filter zones** | Real factories have forbidden and slow zones (near machines, crossings) | Nav2 costmap filters: a keepout mask and a speed mask PGM, generated from new characters in `layout.yaml` |
| **Collision monitor stop/slow polygons** | Only `FootprintApproach` is enabled | Add `PolygonStop` and `PolygonSlow` around the robot, which is standard for industrial safety |
| **Dynamic obstacles** (planned WP9) | The maze is static except for `obstacle.sh` boxes | Spawn moving actors (gz `actor` with trajectories) or a second robot; evaluate MPPI with the `ObstaclesCritic`, and test the AI's reaction to blocked routes |
| **Localisation alternatives** | AMCL on a known 2D map is fine here; a real plant changes | `slam_toolbox` in localisation mode (lifelong mapping), or fused odometry with `robot_localization` (EKF: wheel odom + IMU) |
| **Precise docking** | The charger is a POI with 0.25 m tolerance; real charging contacts need cm accuracy | `opennav_docking` (already configured but unused) with an AprilTag/ArUco on the charger (`apriltag_ros`) |
| **Learned local planning** | Research direction: RL controllers for crowded aisles | Keep MPPI as a safe baseline; compare to an RL local planner trained in Gazebo (e.g. with `gymnasium` + `ros_gz`), with Nav2's collision monitor as a safety layer |
