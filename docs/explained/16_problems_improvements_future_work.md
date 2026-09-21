# 16 — Problems found, improvements, unused technologies, future work

This file collects the findings of the whole guide in one place:
1. **Verified problems** in the code and documentation (with file and line).
2. **Methodological risks** for the thesis claims.
3. **An improvement roadmap for the AI**, in priority order.
4. **Technologies the project does not use**: what they are, why they would or would not fit, and how to add them.
5. **Future research directions.**

Everything in parts 1 and 2 was checked against the code or reproduced by running it on 2026-09-17. Nothing was changed in the code.

---

## 1. Verified problems

### 1.1 Bugs (wrong behaviour)

| # | Severity | Where | Problem | Evidence | Fix |
|---|---|---|---|---|---|
| B1 | **High (thesis fairness)** | `tools/ai/train_ppo.py:62` | `scenarios[rank % len(scenarios)]` with `n_envs = 4` trains PPO on only the **first 4 sorted scenarios**. The committed PPO never saw `one_hot_section` or `worn_robot`, but is evaluated on them | training report lists 6 scenarios with `n_envs: 4`; the env keeps its scenario on reset | random scenario per `reset()`, or `n_envs = len(scenarios)` |
| B2 | Medium | `tools/ai/evaluate.py:94` | `decision_ms` is always 0 unless `--trace`, because `run_episode` only returns decisions when tracing | fresh run printed `0.00` for all 600 episodes | always collect latency in `run_episode`, independent of trace |
| B3 | Medium | `tools/nav/compare_planners.py:165` vs `:223` | `follow()` returns a 2-tuple on rejection, but the caller unpacks 3 values → `ValueError` crash instead of a recorded failed leg | code reading | `return False, "rejected", 0.0` |
| B4 | Medium | `src/robofetch_core/robofetch_core/mission_executor.py:667` | Summary `seed` is `cfg["time"]["seed"]`, not the factory's `seed:=` launch argument → `validate_against_gazebo.py` can replay with the wrong seed | the executor has no `seed` parameter | pass the seed to the executor and record it |
| B5 | Low | `src/robofetch_ai/robofetch_ai/policies/symbolic.py:105-107` | CHARGE tier-2 threshold uses idle energy only, while H3 uses the full simulated route + margin → a **dead band** where all work is forbidden but charging is not prioritised, and the network may choose WAIT away from the charger | constructed state in file 10 §7.3 (battery 17.8 %, `ns` → WAIT:60) | compute the home cost with `simulate_route` exactly like H3 |
| B6 | Low | `src/robofetch_ai/robofetch_ai/service.py:44` | One shared `LiveState` per scenario, mutated per request; FastAPI runs sync endpoints in a thread pool → race with concurrent requests | code reading (no issue with one robot) | per-request `LiveState` or a lock |
| B7 | Low | `src/robofetch_core/robofetch_core/mission_executor.py:350-354` | `nearest_poi()` uses Gazebo **ground truth**, contradicting the "a real robot has none" principle used for arrival checks | code reading | use `amcl_xy` |
| B8 | Low | `src/robofetch_ai/robofetch_ai/policies/rule_based.py:32-46` | `battery_needed()` is dead code, and its `_home_distance(state, location)` ignores `location` | never called | delete or fix |
| B9 | Low | `tools/ai/evaluate.py` | `ppo_unmasked` is selectable but its model file does not exist → crash | `models/` contains only two files | train it or remove the option |

### 1.2 Modelling gaps and inconsistencies

| # | Where | Gap |
|---|---|---|
| M1 | `factory_sim.py:142` | Violations are deduplicated by kind, so a long stretch below reserve costs −20 once. PPO exploits this in `heavy_load` (file 12 §6.8) |
| M2 | `factory_sim.py` `score()` | Units left in buffers or on the robot at shift end count as 0, which rewards emptying buffers early and biases comparisons (file 08 §4.2) |
| M3 | `symbolic.py` | WAIT away from the charger is never checked by any rule. It is the common failure mode in `worn_robot` and `heavy_load` (file 13 §4.3) |
| M4 | `symbolic.py` H3 | The return leg after a PICKUP uses the pre-pickup payload (small underestimate, covered by the 2 % margin) |
| M5 | `factory_sim.py` | No navigation failures, constant speed, no turning or acceleration energy |
| M6 | `robot_model.py` | Constant-power charging without CC/CV taper; no battery ageing |
| M7 | `factory_model.py` | Degraded rate jumps from 1.0 to 0.8 at 60 % health; fault check even while BLOCKED; `time_to_full` uses the true sampled work of the current unit |
| M8 | `factory_env.py` | Per-step discount (not per second); observation lacks absolute rates, unit mass, fault time, cargo |
| M9 | `robot_state_node.py` | `cooldown` duty state is reported but never enforced |
| M10 | `mission_executor.py` | Autonomous shifts can overrun (the shift ends only between actions); CHARGE/WAIT not interruptible |
| M11 | `neural.py` standardisation | Rare binary features (blocked, fault) become 10–16σ inputs in scenarios where they are common |

### 1.3 Stale or wrong documentation in the code

| Where | Says | Reality |
|---|---|---|
| `policies/neural.py:3` | "25 numbers" | 27 features |
| `mission_executor.py:24` | "MultiThreadedExecutor serves the callbacks" | `SingleThreadedExecutor` |
| `robot_state_node.py:15`, `:106` | "tools/ml/", "task manager", "admission control", `order_id` | old RoboFetch project concepts |
| `robot_state_node.py:166` | "Warn once per crossing" | warns every tick |
| `nav2_params.yaml:5-6` | inflation 0.70 → 0.45; max speeds 0.40 m/s | inflation **0.35**; MPPI `vx_max` 0.5 |
| `nav2_params.yaml:120` | parcel snapping | gripper removed in WP1 |
| `params.yaml:96, :122, :142` | `cost_per_second_idle`, `training.batch_size`, `rl.eval_seeds` | read by no code |
| `HANDOVER.md` WP4 | "6 discrete actions, ~28-value observation" | 8 actions, 40 observation values (charging became 3 levels in WP5) |
| `mission_executor.py:670` | `ended_by: plan_finished` | also written when an autonomous shift ends |
| `train_ns.py:137` | `train()` (MSE) | never called; replaced by `train_ranking()` |

---

## 2. Methodological risks for the thesis claims

These come from files 11–13. Each is fixable.

| Claim (HANDOVER WP5) | Risk | What the fresh data says | What to do |
|---|---|---|---|
| "The neuro-symbolic model matches or beats PPO" | PPO had a scenario-coverage bug, a smaller simulation budget (≈ 300 k vs ≈ 1.7 M simulated actions, estimated) and one training seed | Original 6 scenarios: ns − ppo = +0.58 [−0.57, +1.89], **no significant difference**. New scenarios: PPO clearly better in `heavy_load` (unsafely) | Fix B1, train ≥ 3 seeds each, equal-budget learning curves, paired statistics |
| "The network adds judgement" | Ranking accuracy 35 % (≈ chance); rounds 2–3 did not improve | ns − symbolic_only on original 6 = **−0.08 [−1.19, +1.17]** | Present the neural part as "no worse on average, better in some scenarios", or improve it (§3) and show a significant gain |
| "~20 % less energy" | True vs PPO, but ns uses **+18 % to +252 %** more energy than the rule policy, partly because the objective rewards early emptying | balanced: rule 4.6 Wh, ns 8.2 Wh, ppo 10.0 Wh | Fix M2, report the Pareto front (throughput vs energy vs safety), use longer shifts |
| "Never violates safety" | Holds in simulation for the modelled constraints. Depends on model accuracy (±2 % prediction error) and rule coverage | 0/240 episodes with violations for ns and symbolic-only, including unseen scenarios | Keep the claim, state its scope precisely (model-based, one-step, simulated), and add a Gazebo confirmation |
| "What is measured offline drives the robot" | Validated with one 7-action scripted mission | 0.6 % total error on cost; production outcomes not compared | Full-shift comparisons of the same policy in Gazebo vs the fast simulator over several seeds |

---

## 3. AI improvement roadmap

### 3.1 Short term (days): cheap, likely to improve results

1. **Close the rule gaps (B5, M3).** Forbid WAIT away from the charger when DELIVER or any CHARGE is allowed. Make CHARGE tier 2 whenever H3 forbids all work. Re-run `worn_robot` and `heavy_load`, where the effect should be directly visible.
2. **Fix PPO training coverage (B1)** and retrain PPO. Also train `ppo_unmasked` (B9).
3. **Fix the score (M1, M2):** penalise time below reserve, and add a terminal value for leftover units (or evaluate on 8-hour shifts).
4. **Paired analysis in `evaluate.py`** and fix `decision_ms` (B2).
5. **Add the fallback rules as an evaluation baseline** (file 09 §3.4). They are surprisingly strong.

### 3.2 Medium term (weeks): stronger learned component

6. **Retrain the scorer on all 12 scenarios with domain randomisation** and oversampling of hard states (blocked, faults, low battery). Robust input scaling (M11).
7. **Better labels:** more rollout samples where top candidates are close; soft pairwise targets from sample statistics (RankNet-style).
8. **Real DAgger:** aggregate data over rounds; parallelise rollouts; lightweight simulator clone.
9. **Ensemble scorer with a confidence gate:** if models disagree, use the symbolic ranking and say "low confidence" in the explanation.
10. **Shielded PPO** (PPO with the hard rules as a training-time and run-time mask). This is the fairest comparison of "neural ranking methods" behind the same shield.
11. **Constrained RL baseline** (PPO-Lagrangian) to compare "hard shield" vs "learned constraint satisfaction".

### 3.3 Long term (thesis extensions or follow-up)

12. **Decision-time planning:** MCTS with the fast simulator (or a learned model), with the network as prior/value and the shield as the action filter (AlphaZero-style, safe by construction).
13. **Set/attention-based scorer** that sees all candidates together and can plan two-stop trips.
14. **Interruptible options:** re-decide during long CHARGE/WAIT; learn when to stop charging.
15. **Uncertainty-aware safety:** a reserve margin from the prediction-error distribution (conformal prediction), with chance constraints on battery.
16. **Model navigation failures** in the simulator and add failure memory to the state (file 07 §10).

---

## 4. Technologies not used: why and how

The "why not used" column states the documented reason where HANDOVER or the code gives one. Otherwise it gives the trade-off that makes the technology optional here. It does not guess at the author's intent.

### 4.1 AI / decision making

| Technology | What it is | Fit for this project | How to use it here |
|---|---|---|---|
| **MCTS (UCT)** | Tree search that samples futures with a simulator and balances exploration/exploitation (UCB) | Very good: 8 macro-actions, a cheap cloneable simulator. Not used; the project chose learned policies for millisecond decisions | `decide()`: expand legal+allowed actions, rollouts with the symbolic policy, 100–500 simulations, best visit count. Useful as an upper-bound baseline, and live with a time budget |
| **AlphaZero / MuZero** | MCTS guided by a learned policy+value network, trained by self-play | Good, heavy | Use the existing scorer as prior/value; train from MCTS visit counts (distillation) |
| **Constrained RL (CMDP, PPO-Lagrangian, CPO)** | RL maximising reward subject to expected cost ≤ budget | Directly addresses the PPO safety issue (M1, §2) | OmniSafe or a custom Lagrange multiplier on "seconds below reserve" |
| **Shielding (formal)** | A verified runtime monitor that blocks unsafe actions | The symbolic layer is an informal shield | Specify the safety property in LTL; synthesise or verify the shield; keep the Python rules as its implementation |
| **Offline RL (CQL, IQL)** | Learn from logged data without new interaction | Logs of rule/ns episodes exist | `d3rlpy` with discrete CQL on (obs, action, reward) from `evaluate.py` traces |
| **Imitation learning (BC, DAgger)** | Learn from an expert's actions | The rollout teacher is an expert; DAgger is partly used | Distil MCTS or a long-horizon rollout teacher into the scorer |
| **Recurrent / Transformer policies** | Memory over past observations | Partial observability (faults, trends) | `sb3_contrib.RecurrentPPO`; a small Transformer over the last N decisions |
| **Graph neural networks** | Networks over graphs of entities | Future: many stations, multiple robots | Nodes = POIs/robots/sections, edges = path lengths; PyTorch Geometric |
| **LLM-based planning / explanation** | Language models proposing plans or phrasing explanations | Not needed for control. Risky for safety-critical decisions. Possibly useful for **explanations to operators** | Keep decisions symbolic+neural; an LLM (e.g. via the Anthropic API) can turn the structured verdicts and scores into operator-friendly summaries, *never* into actions |
| **Classical OR (MILP, CP-SAT, VRP solvers)** | Mathematical optimisation of routes and schedules | A strong non-learning baseline; rolling-horizon planning fits well | OR-Tools CP-SAT: variables = trip sequence over the next 15–30 min with expected production, battery constraints, objective = lost + energy |
| **Bayesian optimisation / hyperparameter search** | Sample-efficient tuning of expensive black boxes | Many hand-set values (rule thresholds, training settings) | Optuna with pruning; objective = paired validation score |
| **Probabilistic programming / uncertainty** | Posterior distributions over model parameters | Parameters are estimates; safety margin is fixed | PyMC calibration of energy/thermal parameters from logs; conformal intervals for trip energy |
| **Neuro-symbolic frameworks** (DeepProbLog, Scallop, Logic Tensor Networks) | Differentiable logic with neural predicates | Research-level alternative to "shield + ranker" | Encode tiers and soft priorities as weighted rules; learn rule weights end to end |
| **Declarative rules** (ASP/clingo, Datalog, PDDL) | Rules as data, solver-evaluated | Makes rules auditable and verifiable | Hard rules as ASP constraints; action preconditions in PDDL; `z3` to prove rule properties |

### 4.2 Robotics / simulation

| Technology | Fit | How |
|---|---|---|
| **ros2_control + diff_drive_controller** | The real-robot interface (the Gazebo DiffDrive plugin is sim-only) | `gz_ros2_control`, controller YAML, spawner nodes |
| **Nav2 docking server + AprilTags** | Precise charging instead of a 0.25 m tolerance | `opennav_docking` (already configured, unused) + `apriltag_ros` |
| **Nav2 keepout/speed filters, collision monitor zones** | Industrial safety zones | Filter masks generated from new layout characters |
| **robot_localization (EKF), slam_toolbox** | Robust localisation in changing plants | Fuse wheel odometry + IMU; lifelong mapping |
| **Open-RMF** | Multi-robot fleet management, traffic, doors, lifts | Fleet adapter around the executor; tasks from the decision layer |
| **Isaac Sim / Isaac Lab, Webots** | High-fidelity or GPU-parallel simulation | Only if perception or massive parallel RL matters; the fast simulator already covers decision training |
| **Dynamic obstacles / actors** | Realistic aisles (planned WP9) | Gazebo actors or a second robot; MPPI `ObstaclesCritic` |
| **rosbag2 (MCAP), Foxglove** | Replay, debugging, visual analysis | Record per run; Foxglove Studio via `foxglove_bridge` |

### 4.3 Software engineering / MLOps

| Technology | Fit | How |
|---|---|---|
| **Docker / devcontainer** | Fragile venv+ROS setup | Image with ROS Jazzy, Gazebo Harmonic, a pinned lock file |
| **CI (GitHub Actions)** | 161 tests run only manually | Build + pytest + short evaluation smoke test |
| **Experiment tracking (MLflow, W&B)** | Training runs are timestamped JSON | Log configs, metrics, artefacts, git commit |
| **ONNX / onnxruntime** | Lightweight, portable inference | Export the scorer and PPO actor; the service loads ONNX |
| **Pydantic / Hydra config** | Typed, validated, composable config | Replace manual key checks; sweeps from the CLI |
| **Prometheus + Grafana** | Live metrics and alerts | Instrument service latency, fallback rate, battery |
| **rliable** | Robust RL statistics across tasks | IQM and performance profiles for the 12-scenario results |

---

## 5. Future work

### 5.1 Directly extending this thesis

1. **Longer and continuous operation.** 8-hour shifts, battery ageing, and maintenance scheduling (condition < 30 % → maintenance trip) make "resource efficiency" a long-horizon problem where charging strategy dominates.
2. **Navigation-aware decisions.** Use live Nav2 path costs or a learned travel-time model instead of the static matrix; react to blocked corridors; include the WP8 planner choice as a decision-layer parameter (e.g. Theta* for long trips).
3. **Energy model with turning and acceleration**, calibrated from Gazebo torques, so path shape matters and planner comparisons show real energy differences.
4. **Gazebo-level evaluation** of the final policies (a subset of scenarios and seeds) to confirm the fast-simulator ranking.
5. **Human factors of explanations:** a small study of whether operators understand or trust the robot more with rule-grounded explanations than with PPO probabilities.

### 5.2 Beyond this thesis

6. **Multi-robot coordination:** task allocation (auction/market-based or MARL), shared chargers, traffic at narrow passages; Open-RMF integration.
7. **Two-way logistics:** robots also supply input material, so lines can starve as well as block.
8. **Real hardware transfer:** TurtleBot-class robot, real battery telemetry replacing the model (the model then becomes a *predictor* checked against measurements, i.e. a digital twin with online calibration).
9. **Certification perspective:** map the hard rules to safety requirements (ISO 3691-4 for driverless industrial trucks), and separate software rules (performance/efficiency) from the safety-rated layer.
10. **Continual learning with a shield:** update the scorer from live operation data while the symbolic layer guarantees safety during learning (safe online learning).
