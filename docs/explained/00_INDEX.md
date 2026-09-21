# Project guide: Towards Autonomous Industrial Mobility

This folder explains the whole workspace, one section per file. It covers what each part does, the concepts behind it, the exact code that does it, worked examples with real numbers from this repository, and the weak spots. Each section ends with improvements and technologies the project could use.

Everything here was checked against the source code on 2026-09-17 (commit `14a4251` plus the uncommitted WP8 planner work). The numbers in the examples come from running the project's own code. They are not made up. When a number comes from `HANDOVER.md` rather than from a fresh run, the text says so.

---

## Reading order

If your main interest is the AI, read 01 → 05 → 06 → 08 → 09 → 10 → 11 → 12 → 13 → 16 first, then fill in the rest.

| # | File | What it covers |
|---|------|----------------|
| 01 | [01_big_picture_and_architecture.md](01_big_picture_and_architecture.md) | The problem, the scenario, all 10 packages, how data flows, one autonomous run from start to end |
| 02 | [02_ros2_foundations.md](02_ros2_foundations.md) | ROS 2 concepts as this project uses them: nodes, topics, services, actions, messages, TF, sim time, lifecycle, launch, colcon |
| 03 | [03_world_map_and_path_generation.md](03_world_map_and_path_generation.md) | `layout.yaml` → Gazebo world, occupancy map, POIs, path-length matrix; distance transform, Dijkstra, string pulling, pinch detection |
| 04 | [04_simulation_robot_and_navigation.md](04_simulation_robot_and_navigation.md) | URDF/xacro robot, Gazebo Harmonic, ros_gz bridge, Nav2: AMCL, costmaps, planners, MPPI, behaviour tree; the WP8 planner comparison |
| 05 | [05_robot_energy_thermal_wear_model.md](05_robot_energy_thermal_wear_model.md) | The battery, energy, charging, temperature and wear equations, with worked examples |
| 06 | [06_factory_production_model_and_scenarios.md](06_factory_production_model_and_scenarios.md) | Production sections: Gamma cycle times, buffers, blocking, machine health, random faults, all 12 scenarios, utilisation analysis |
| 07 | [07_mission_execution_and_recovery.md](07_mission_execution_and_recovery.md) | Action vocabulary, cost prediction, the mission executor, Nav2 recovery ladder, arrival verification, emergency stop, logs |
| 08 | [08_fast_simulator_and_gym_environment.md](08_fast_simulator_and_gym_environment.md) | `FactorySim`, legal actions (masks), objective/reward, the semi-Markov decision problem, the Gymnasium wrapper, sim-vs-Gazebo validation |
| 09 | [09_policy_interface_rule_based_and_fallback.md](09_policy_interface_rule_based_and_fallback.md) | The `Policy` interface, the rule-based reference policy, the fallback policy, and where the rules fail |
| 10 | [10_neuro_symbolic_model.md](10_neuro_symbolic_model.md) | **The thesis model**: hard rules, priority tiers, 27 features, the neural scorer, residual scoring, explanations, ablation modes |
| 11 | [11_training_the_neural_scorer.md](11_training_the_neural_scorer.md) | How the scorer is trained with no labels: look-ahead rollouts, Monte-Carlo value, common random numbers, policy iteration, DAgger, ranking loss |
| 12 | [12_ppo_reinforcement_learning.md](12_ppo_reinforcement_learning.md) | RL from scratch: MDP, policy gradient, actor-critic, GAE, PPO clipping, entropy bonus, action masking, the training setup and its problems |
| 13 | [13_evaluation_results_and_analysis.md](13_evaluation_results_and_analysis.md) | How policies are compared (paired seeds, bootstrap CI), the stored WP5 results, a **fresh 12-scenario run**, and what the numbers really say |
| 14 | [14_live_integration_service_dashboard_launch.md](14_live_integration_service_dashboard_launch.md) | The decision service (FastAPI), `LiveState`, fallback under failure, the dashboard, `mission.launch.py`, `run.sh`, `stop.sh` |
| 15 | [15_configuration_tooling_and_tests.md](15_configuration_tooling_and_tests.md) | `params.yaml` key by key, config loading and typo protection, `param_report.py`, all 161 tests, environment setup |
| 16 | [16_problems_improvements_future_work.md](16_problems_improvements_future_work.md) | Verified bugs and inconsistencies, AI improvements in priority order, technologies not used (why and how), future work |

---

## The project in one paragraph

A small differential-drive robot works alone in a simulated factory hall shaped like a maze. Three production sections (A, B, C) put finished parts into small output buffers at 20, 30 and 5 units per hour. If a buffer fills up, that line **stops** and its production is **lost**. The robot must collect parts, carry them to **one delivery point**, and keep its battery alive at **one charger**. Every few minutes it must decide *what to do next*: pick up from A, B or C, deliver, charge to 40/70/100 %, or wait. The thesis compares three ways of making that decision. The first is a hand-written **rule-based** policy. The second is a **neuro-symbolic** model, where hard safety rules and priority rules are combined with a small neural network that ranks the remaining options. The third is a **PPO** deep reinforcement-learning agent. The decision is made by the AI. The driving is done by ROS 2 Nav2 inside Gazebo. A fast pure-Python copy of the factory is used to train and evaluate the AI thousands of times faster than Gazebo.

---

## Glossary (short; each term is explained properly in its section)

| Term | Meaning here | Section |
|---|---|---|
| **POI** | Point of interest: `A`, `B`, `C`, `delivery`, `charger`, each a pose (x, y, yaw) | 03 |
| **Path matrix** | Pre-computed maze driving distance between every pair of POIs | 03 |
| **Section** | One production line: machine + output buffer | 06 |
| **BLOCKED** | Buffer full, machine stopped, production being lost | 06 |
| **DEGRADED** | Machine health below 60 %, runs slower | 06 |
| **FAULT** | Machine broken, under repair | 06 |
| **Action** | `PICKUP:<A/B/C>`, `DELIVER`, `CHARGE:<%>`, `WAIT:<s>`. One action = a whole trip | 07 |
| **Shift / episode** | One simulated working period, 3600 s by default | 08 |
| **Legal actions / action mask** | The actions that make sense right now (e.g. no DELIVER with an empty load) | 08 |
| **Objective / score** | +1 per unit delivered, −1 per unit lost, −0.5 per Wh, −20 per safety violation | 08 |
| **Policy** | Anything that maps a state to an action | 09 |
| **Hard rule** | A symbolic rule that *forbids* an action, and nothing can override it | 10 |
| **Priority tier** | 2 = unblock a stopped line, 1 = prevent a stop, 0 = routine | 10 |
| **Scorer** | The 27→64→64→1 MLP that scores one candidate action | 10 |
| **Rollout / look-ahead** | Cloning the simulator, taking an action, playing the shift on to measure what it was worth | 11 |
| **PPO** | Proximal Policy Optimization, a policy-gradient RL algorithm | 12 |
| **Ablation** | Removing one part of a model to measure what it contributes (`ns_neural_only`, `ns_symbolic_only`) | 10, 13 |
| **Fallback** | Tiny rule set inside the executor, used when the AI service does not answer | 09, 14 |
| **Sim time** | The Gazebo clock (`/clock`). Every duration in this project is simulation seconds | 02 |
| **WP** | Work package of the thesis plan (WP0–WP9), logged in `HANDOVER.md` | 01 |
