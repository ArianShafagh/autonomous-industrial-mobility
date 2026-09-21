# Thesis data index

Where every result lives, which files are **final**, and what each number means — for writing the
thesis without running anything. Everything listed here is committed in this repository.

> **Authoritative sources, in this order:** (1) this file for the final numbers, (2) `HANDOVER.md`
> for how each number was produced and why decisions were made, (3) the raw files below.
> `docs/explained/` was written on 2026-09-17, *before* the final WP9 results: its architecture and
> method chapters are useful, but its result numbers are **superseded** (see "Stale material").

## 1. Final headline results

### Fast simulator (tier 1) — the main evaluation
**File:** `tools/ai/results/summary_20260920_210847.md` (tables), `episodes_20260920_210847.csv` (one
row per shift). 9 policies × 12 scenarios × 30 seeds = 3240 shifts, 1-hour shifts, test seeds
5000–5029 (never used in training or model selection). p = paired Wilcoxon signed-rank vs `ns`.

`score = delivered units − lost production − 0.5 × Wh − 20 × safety violations`

| policy | what it is | score [95 % CI], all 12 scenarios | Wh | shifts with violation | Δ vs ns | p |
|---|---|---|---|---|---|---|
| **ns** | **the thesis model**: rules + bounded neural scorer | **53.81 [51.9, 55.7]** | 8.42 | **0/360** | — | — |
| ns_unbounded | WP5 model, neural correction not bounded | 51.56 | 8.08 | 0/360 | −2.25 | <0.001 |
| ns_estimate_only | rules + symbolic estimate, network off | 52.85 | 8.77 | 0/360 | −0.96 | <0.001 |
| ns_symbolic_only | rules alone, cheapest-per-unit ranking | 52.91 | 8.71 | 0/360 | −0.90 | <0.001 |
| ns_neural_only | the network alone, no rules | 51.81 | 8.56 | **57/360** | −2.00 | 0.109 |
| rule | simple reference policy | 41.66 | 4.97 | 0/360 | −12.15 | <0.001 |
| ppo | deep RL (MaskablePPO), 6 training scenarios | 52.35 | 10.21 | **53/360** | −1.46 | <0.001 |
| ppo_unmasked | PPO without the action mask | 28.19 | 7.66 | 42/360 | −25.62 | <0.001 |
| ppo_wp6 | WP6 PPO (trained on only 4 scenarios) | 53.75 | 9.95 | **51/360** | −0.06 | 0.121 |

**Generalisation** — the 6 scenarios added after training (`aged_battery`, `heavy_load`,
`heavy_parts`, `hot_factory`, `section_breakdown`, `small_buffers`), 180 shifts each:

| policy | score, unseen | shifts with violation | Δ vs ns | p |
|---|---|---|---|---|
| **ns** | **47.20 [46.1, 48.3]** | **0/180** | — | — |
| ns_unbounded | 43.33 | 0/180 | −3.87 | <0.001 |
| ns_symbolic_only | 45.66 | 0/180 | −1.54 | <0.001 |
| ppo | 44.83 | **49/180** | −2.37 | <0.001 |
| ns_neural_only | 50.84 (unsafe) | **30/180** | +3.64 | 0.034 |

A higher score reached with violations is not a valid result: the penalty is 20 per violation
*kind*, not per minute below the reserve, so an unsafe policy can still out-score a safe one.

### Gazebo (tier 2) — does the fast simulator tell the truth?
- **20-minute shifts, 12 runs** — `tools/ai/results/gazebo_20260917_150533.csv`, figure
  `fig_gazebo_vs_sim.png`. Over the 11 runs that finished: **energy +0.2 %** (3.24 vs 3.24 Wh),
  score −5.7 %, delivered −5.1 %, 0 violations on either side. The gap is time, not logic: real
  driving loses a few seconds per leg.
- **60-minute shifts, `low_battery_start`, seed 5000** — `tools/ai/results/gazebo_60min.csv`:

  | model | Gazebo score | delivered | Wh | minimum battery | violations |
  |---|---|---|---|---|---|
  | **ns** | **55.16** | 61 | 7.36 | **17.0 %** | **0** |
  | ppo | 37.79 | 62 | 8.42 | **9.4 %** | **1** (below the 15 % reserve for 4 actions) |

### Global path planner (WP8 + WP9)
- **WP8, short 7-leg tours** — `tools/nav/results/` (`planners_*.csv`, `planners_drive_*.csv`).
  5 repeats: ThetaStar fastest (147.8 ± 0.9 s vs 152.7 ± 1.9 s, p = 0.001) and least turning.
- **WP9, full 20-minute shifts, 5 per planner** — `tools/ai/results/plannercheck_*_r*.csv` + `.log`:
  ThetaStar faulty in 3 of 5 (planner stalls down to 1.1 Hz, refusals *"Either of the start or
  goal pose are an obstacle!"*), NavfnDijkstra 0 of 5; scores equal (18.07 vs 17.75, sd ~1.4).
  Over all Gazebo shifts: **ThetaStar faulty in 6 of 17, NavFn in 0 of 5**. Default: NavfnDijkstra.

## 2. Figures (`tools/ai/results/`, regenerated from the final data)

| file | shows |
|---|---|
| `fig_score_by_scenario.png` | score per policy per scenario, 95 % CI; hatched = shifts with violations |
| `fig_violations.png` | share of shifts with a safety violation per policy |
| `fig_energy_throughput.png` | delivered units vs Wh per delivered unit |
| `fig_gazebo_vs_sim.png` | Gazebo score vs fast-simulator score, one point per shift |

Regenerate: `venv/bin/python tools/ai/plot_results.py --episodes tools/ai/results/episodes_20260920_210847.csv --gazebo tools/ai/results/gazebo_20260917_150533.csv`

## 3. Where each story in the thesis comes from (all in `HANDOVER.md`)

| topic | HANDOVER section |
|---|---|
| maze, single source of truth, the hidden 0.5 m pinch | WP1 |
| factory model (20/30/5 units per hour), robot energy/thermal/wear | WP2 |
| mission executor, navigation recovery ladder | WP3 |
| fast simulator validated against Gazebo (within 0.6 %) | WP4 |
| neuro-symbolic model, ranking loss, PPO, ablations | WP5 + WP6 |
| live decision service, fallback, dashboard | WP7 |
| planner comparison on short tours | WP8 |
| PPO trained on 4/6 scenarios (fixed); neural part overruling its rules in `heavy_load`; the bounded correction | WP9, sections 1–5 |
| findings list for the discussion chapter | WP9, "Findings for the thesis" |
| Gazebo validation, navigation faults, planner check | WP9, "Tier 2" + "Planner check" |
| H5 rule, `worn_robot` batching trade-off, energy-accounting limitation, 60-min result | WP9 follow-up |

## 4. Raw data

| path | content |
|---|---|
| `tools/ai/results/episodes_*.csv` | one row per simulated shift; the **final** one is `episodes_20260920_210847.csv`, earlier ones are WP4–WP9 intermediate runs cited in HANDOVER |
| `tools/ai/results/summary_*.md` | per-scenario tables; **final** `summary_20260920_210847.md`; `summary_20260917_150043.md` is the evaluation *before* the bounded correction (the `heavy_load` failure) |
| `tools/ai/results/train_*.json`, `wp9_train_ppo_*.log` | training runs (NS rounds, PPO configs and times) |
| `tools/ai/results/gazebo_*.csv`, `gazebo_*.log`, `wp9_*.log` | Gazebo batches and their full ROS logs |
| `tools/ai/checkpoints/ppo_policy_wp6.zip` | the WP6 PPO model (policy `ppo_wp6`) |
| `logs/run_*_mission_summary.yaml` | every Gazebo mission: actions, energy, navigation stats, prediction error |
| `logs/run_*_mission.csv` | per action: predicted vs measured time, distance, energy, battery |
| `logs/run_*_robot.csv`, `run_*_section_*.csv` | 1 Hz telemetry of the robot and each section |
| `src/robofetch_factory/config/params.yaml` | every number of the model (battery 22 Wh, reserve 15 %, rates, objective weights, training settings, `max_correction: 1.0`) |
| `src/robofetch_factory/config/scenarios/*.yaml` | the 12 scenarios with descriptions |

## 5. Stale material (do not cite without checking)
- `docs/explained/` (2026-09-17): written before the bounded correction, H5, the Gazebo batches and the planner switch. In particular `10_neuro_symbolic_model.md`, `13_evaluation_results_and_analysis.md` and `16_problems_improvements_future_work.md` describe the *unbounded* model and pre-WP9 numbers, and the planner default is Theta\* there.
- `gazebo_20260917_150533_before_pause_cleanup.csv`: raw copy kept for traceability; use `gazebo_20260917_150533.csv`.
- `gazebo_smoke.csv`: a 3-minute smoke test of the batch tool, not a result.
