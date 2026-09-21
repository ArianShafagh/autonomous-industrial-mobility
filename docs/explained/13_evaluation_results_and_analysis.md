# 13 — Evaluation: method, results, and what the numbers really say

Files:
- `tools/ai/evaluate.py` (131 lines)
- Results: `tools/ai/results/episodes_<time>.csv` (git-ignored)
- Stored results: `HANDOVER.md` WP4 and WP5/6
- **Fresh run for this guide:** `tools/ai/results/episodes_20260917_093244.csv` (5 policies × 12 scenarios × 10 seeds = 600 episodes, 77 s)

```bash
venv/bin/python tools/ai/evaluate.py --policies rule ns ns_symbolic_only ns_neural_only ppo --seeds 10
venv/bin/python tools/ai/evaluate.py --policies ns --scenarios low_battery_start --seeds 1 --trace
```

---

## 1. The evaluation procedure

```python
cfg = load_config("balanced", CONFIG_DIR)               # used to BUILD the policies (see note)
for policy_name in args.policies:
    policy = build_policy(policy_name, cfg)
    for scenario in scenarios:
        for seed in range(args.seeds):                  # seeds 0..N-1, the SAME for every policy
            policy.reset()
            summary, decisions = run_episode(policy, scenario, seed, CONFIG_DIR,
                                             shift_duration_s=args.shift_s,
                                             trace=args.trace and seed == 0)
            summary["policy"] = policy_name
            summary["violation_count"] = len(summary["violations"])
            summary["decision_ms"] = mean(latency of decisions) if decisions else 0.0
            rows.append(summary)
write CSV; print per (policy, scenario): mean of each metric + 95 % bootstrap CI of the score
print "SAFETY VIOLATIONS in k of n episodes" when k > 0
```

### 1.1 Metrics

| Metric | Meaning | Better |
|---|---|---|
| `score` | the objective: delivered − lost − 0.5·Wh − 20·violations | higher |
| `delivered_units` | units delivered before the shift ended | higher |
| `lost_units` | production lost while lines were BLOCKED | lower |
| `energy_wh` | energy drawn from the battery | lower |
| `wh_per_unit` | energy / delivered | lower |
| `distance_m` | distance driven | (context) |
| `charge_trips` | number of CHARGE actions | (context) |
| `min_battery_percent` | lowest battery during the shift | higher is safer |
| `actions` | decisions taken | (context) |
| `decision_ms` | mean decision latency | lower (**buggy, see §1.3**) |

### 1.2 Concept: bootstrap confidence interval

```python
def bootstrap_ci(values, samples=2000, alpha=0.05, seed=0):
    means = sorted(mean(rng.choices(values, k=len(values))) for _ in range(samples))
    return means[int(0.025 * 2000)], means[int(0.975 * 2000) - 1]
```

**Percentile bootstrap:** resample the n episode scores *with replacement* 2000 times, compute each resample's mean, and take the 2.5th and 97.5th percentiles as a 95 % CI for the mean. It needs no normality assumption. With n = 10–20 it is approximate (percentile intervals tend to be too narrow for small n).

### 1.3 Issues in the evaluation tool

1. **"Paired" in design, not in analysis.** The docstring says *"Policies are compared on the SAME seeds, so the comparison is paired."* The seeds are indeed shared, but the tool prints a separate CI **per policy**. Two overlapping per-policy CIs do not mean "no difference", and non-overlapping ones overstate it. The correct analysis takes **per-seed differences** between two policies and puts a CI or test on those (done in §3.3 below).
2. **`decision_ms` is always 0.00** unless `--trace` is set. `run_episode` only fills `decisions` when `trace=True`, and trace is only on for seed 0 when `--trace` is given. The printed latency column is meaningless. (Measured separately: ns ≈ 1.2 ms, symbolic-only ≈ 0.8 ms, PPO ≈ 1.6 ms per decision; the rule policy does not record latency.)
3. **Policies are built with the balanced config** (`cfg = load_config("balanced")`), and the episode then runs in another scenario. For `NeuroSymbolicPolicy` this only affects `cfg["mission"]` (urgency horizon, safety margin, objective weights in `symbolic_estimate`), which no scenario overrides, so results are unaffected. The robot parameters used by the rules come from `sim.p`, which is scenario-correct. It is still fragile.
4. **Same seeds ≠ same factory** once actions differ (file 06 §9), so pairing reduces but does not remove noise.
5. **`ppo_unmasked`** is listed but has no trained model, so selecting it crashes.

---

## 2. Stored results (HANDOVER.md)

### 2.1 WP4: rule-based reference, 20 seeds

| Scenario | Score | Delivered | Lost | Energy Wh | Charge trips | Min battery % |
|---|---|---|---|---|---|---|
| balanced | 46.7 | 49.1 | 0.0 | 4.86 | 9.1 | 96.1 |
| one_hot_section | 74.0 | 77.3 | 0.0 | 6.44 | 11.2 | 93.6 |
| high_demand | 94.4 | 98.2 | 0.02 | 7.53 | 8.7 | 85.6 |
| fault_burst | 31.3 | 33.0 | 0.0 | 3.41 | 7.2 | 96.5 |
| worn_robot | 46.5 | 49.1 | 0.0 | 5.21 | 9.1 | 95.7 |
| low_battery_start | **9.6** | 32.3 | **21.6** | 2.23 | 2.0 | 27.1 |

(These numbers are from before charging became three levels in WP5. The later WP5 table shows the rule policy at −4.9 in `low_battery_start`.)

### 2.2 WP5/6: all models, 20 seeds, original 6 scenarios (`logs/wp5_eval6.txt`)

| Scenario | rule | **ns** | ns_symbolic_only | ppo |
|---|---|---|---|---|
| balanced | 46.8 | **47.9** | 46.3 | 46.8 |
| one_hot_section | 72.8 | **74.8** | 72.9 | 74.4 |
| high_demand | 93.2 | **96.9** | 96.7 | 96.4 |
| fault_burst | 31.0 | **32.0** | 30.7 | 31.5 |
| low_battery_start | −4.9 | 49.5 | **54.0** | 48.4 |
| worn_robot | **46.6** | 43.9 | 45.4 | 46.4 |

| Policy | Episodes with violations | Lost production | Energy (balanced) | Explains |
|---|---|---|---|---|
| ns | 0/120 | 0.00 everywhere | 8.0 Wh | yes |
| ns_symbolic_only | 0/120 | ≤ 0.52 | 8.4 Wh | yes |
| rule | 0/120 | ≤ 27.8 | 4.9 Wh | yes |
| ns_neural_only | 20/20 in low battery | ≤ 28.5 | 8.4 Wh | no |
| ppo | 5/20 in low battery | ≤ 0.40 | 10.1 Wh | no |

The HANDOVER headline: *"the neuro-symbolic model matches or beats PPO on throughput while using ~20 % less energy, never violates safety, and explains every decision."*

---

## 3. Fresh evaluation for this guide (2026-09-17)

It uses the same code and the same committed models, **all 12 scenarios**, and **10 seeds** (0–9). The six scenarios added after training (`aged_battery`, `heavy_load`, `heavy_parts`, `hot_factory`, `section_breakdown`, `small_buffers`) were never seen by either learned model, so they test **generalisation**.

### 3.1 Mean score (95 % bootstrap CI in brackets)

| Scenario | rule | **ns** | ns_symbolic_only | ns_neural_only | ppo |
|---|---|---|---|---|---|
| *original scenarios* | | | | | |
| balanced | 45.18 [41.3, 48.2] | **47.11** [43.0, 50.4] | 45.04 | 46.79 | 46.20 |
| fault_burst | 30.15 | 31.01 | 30.49 | 30.98 | 30.56 |
| high_demand | 91.67 | **96.73** | 95.93 | 96.28 | 94.69 |
| low_battery_start | −3.62 | 49.66 | **52.93** | 42.25 ⚠ | 46.73 ⚠ |
| one_hot_section | 72.64 | **74.42** | 71.74 | 74.23 | 74.14 |
| worn_robot | **46.11** | 42.34 | 45.64 | 41.93 | 45.48 |
| *new (unseen) scenarios* | | | | | |
| aged_battery | 44.72 | **47.27** | 43.97 | 45.40 | 41.29 ⚠ |
| heavy_load | −22.41 | 35.54 | 49.79 | 42.43 ⚠ | **71.51** ⚠ |
| heavy_parts | 44.42 | **45.84** | 43.77 | 43.37 | 45.79 |
| hot_factory | 45.07 | 44.94 | **46.10** | 45.12 | 45.77 |
| section_breakdown | 29.14 | 32.15 | 31.43 | **32.23** | 31.33 |
| small_buffers | 43.68 | **43.91** | 41.20 | 41.20 | 43.70 |

⚠ = at least one episode with a safety violation.

### 3.2 Safety, energy, loss

**Episodes with ≥ 1 safety violation (of 10):**

| Scenario | rule | ns | sym | neural | ppo |
|---|---|---|---|---|---|
| aged_battery | 0 | 0 | 0 | 0 | **2** |
| heavy_load | 0 | 0 | 0 | **10** | **9** |
| low_battery_start | 0 | 0 | 0 | **6** | **3** |
| all others | 0 | 0 | 0 | 0 | 0 |
| **total /120** | **0** | **0** | **0** | **16** | **14** |

**Mean energy (Wh per shift):**

| Scenario | rule | ns | sym | neural | ppo |
|---|---|---|---|---|---|
| balanced | 4.64 | 8.18 | 8.32 | 8.26 | 9.99 |
| high_demand | 6.96 | 9.94 | 9.70 | 9.94 | 10.58 |
| low_battery_start | 1.84 | 6.48 | 6.90 | 7.41 | 7.32 |
| worn_robot | 5.18 | 7.32 | 8.93 | 7.30 | 10.64 |
| aged_battery | 4.36 | 7.46 | 7.86 | 7.02 | 9.83 |
| heavy_load | 3.25 | 7.69 | 7.45 | 10.81 | 10.43 |
| small_buffers | 6.55 | 10.18 | 8.60 | 9.72 | 10.06 |

**Lost production** was essentially zero for all learned/symbolic policies except in `heavy_load`: ns 26.0, sym 20.6, neural 4.6, **ppo 2.0**, rule 57.0. In `low_battery_start` the rule policy lost 26.6. In `small_buffers` losses were small (0.5–2.8).

### 3.3 Paired comparisons (the analysis `evaluate.py` does not do)

For each seed, difference = score(ns) − score(other). Shown are the mean difference, a 95 % bootstrap CI over the 10 seeds, and the Wilcoxon signed-rank p-value. **Bold** marks CIs that exclude 0.

| Scenario | ns − rule | ns − symbolic_only | ns − ppo |
|---|---|---|---|
| aged_battery | **+2.55 [+0.30, +4.25]** p=0.064 | **+3.30 [+0.77, +6.48]** p=0.010 | **+5.98 [+2.04, +10.94]** p=0.006 |
| balanced | **+1.93 [+0.88, +2.99]** p=0.014 | +2.07 [−0.21, +5.31] p=0.23 | +0.91 [−0.29, +2.18] p=0.19 |
| fault_burst | +0.85 [−0.69, +2.37] | +0.51 [−0.62, +1.83] | +0.44 [−0.51, +1.36] |
| heavy_load | **+57.95 [+46.5, +68.1]** p=0.002 | **−14.25 [−22.9, −5.5]** p=0.014 | **−35.98 [−47.1, −23.5]** p=0.002 |
| heavy_parts | +1.41 [−0.14, +2.83] | +2.06 [−0.39, +5.02] | +0.05 [−0.60, +0.82] |
| high_demand | **+5.06 [+1.38, +9.61]** p=0.027 | +0.80 [−1.16, +2.70] | +2.04 [−1.40, +6.41] |
| hot_factory | −0.13 [−1.58, +1.34] | **−1.16 [−2.22, −0.13]** p=0.084 | −0.83 [−1.86, +0.24] |
| low_battery_start | **+53.28 [+45.8, +59.7]** p=0.002 | **−3.27 [−6.29, −0.25]** p=0.064 | +2.94 [−1.91, +8.08] |
| one_hot_section | **+1.79 [+0.29, +3.65]** p=0.084 | +2.68 [−0.34, +6.99] | +0.29 [−0.80, +1.30] |
| section_breakdown | **+3.01 [+1.30, +5.04]** p=0.004 | +0.72 [−0.43, +1.87] | +0.82 [−0.39, +1.95] |
| small_buffers | +0.23 [−1.26, +1.75] | +2.71 [−0.27, +5.86] | +0.21 [−0.76, +1.30] |
| worn_robot | **−3.77 [−5.53, −2.24]** p=0.002 | **−3.30 [−4.43, −2.26]** p=0.002 | **−3.14 [−4.73, −1.63]** p=0.010 |

Pooled (60 paired episodes each):

| Comparison | Original 6 scenarios | New 6 scenarios |
|---|---|---|
| ns − rule | **+9.86 [+5.00, +15.11]** | **+10.84 [+5.70, +16.69]** |
| ns − ns_symbolic_only | **−0.08 [−1.19, +1.17]** | −1.10 [−3.46, +1.01] |
| ns − ppo | +0.58 [−0.57, +1.89] | −4.96 [−9.42, −1.09] (driven by heavy_load) |
| ns − ns_neural_only | **+1.47 [+0.38, +2.76]** | −0.02 [−1.88, +1.62] |

(The p-values are uncorrected for the 36 scenario-level tests. With a Holm correction only the large effects, `heavy_load`, `low_battery_start` vs rule and `worn_robot`, would clearly survive.)

---

## 4. Interpretation

### 4.1 What is supported by the evidence

1. **The learned/symbolic policies clearly beat the hand-written rule policy**, overall by about +10 points per shift, mostly because the rule policy collapses in `low_battery_start` (+53) and `heavy_load` (+58). In nominal scenarios the advantage is small (+1 to +5) but often significant.
2. **The symbolic shield delivers exactly what it promises.** `ns` and `ns_symbolic_only` had **0 violations in 120 + 120 episodes**, including the six unseen scenarios. Without it, the same network violated safety in 16/120 episodes and PPO in 14/120. This is the thesis's strongest, cleanest result, and it is structural (file 10 §9).
3. **Explanations for every decision**, with 0 unexplained decisions in the tests for all 12 scenarios.

### 4.2 What is *not* supported (or needs rewording)

1. **"The network adds value over the rules."** On the original 6 scenarios, `ns` − `ns_symbolic_only` = **−0.08 [−1.19, +1.17]**. With 10 seeds there is **no measurable benefit** from the neural scorer on average. It helps in some scenarios (aged_battery +3.3) and hurts in others (low_battery_start −3.3, worn_robot −3.3, heavy_load −14.3). The HANDOVER table (20 seeds) shows the same pattern: `ns` won 4 of 6 scenarios by small margins, and symbolic-only won `low_battery_start` by 4.5.
2. **"Matches or beats PPO."** It matches PPO on the original scenarios (+0.58, CI includes 0). It does **not** beat it significantly, and it is clearly beaten in `heavy_load` (−36) and `worn_robot` (−3.1). Keep in mind PPO's scenario-coverage bug and smaller simulation budget (file 12 §6). In fairness, PPO's `heavy_load` score is bought with 9/10 unsafe episodes.
3. **"Uses ~20 % less energy than PPO."** That is true (≈ 7–8 Wh vs ≈ 10 Wh), but **`ns` uses more energy than the rule policy in every scenario**: +18 % (heavy_parts) to +252 % (low_battery_start), and +40–95 % in 9 of 12 scenarios (e.g. balanced 8.2 vs 4.6 Wh), for small score gains, mostly from 1-unit "milk runs" and an objective that rewards emptying buffers before the shift ends (file 08 §4.2). For a thesis titled *resource-efficient navigation* this needs to be discussed openly, or the objective changed.

### 4.3 Explaining the failures (from traces)

**`worn_robot` (ns significantly worse than all).** On seed 0 the model took **43 WAIT:60 actions** out of 60, many of them *at the delivery point* with the explanation text itself saying "waiting away from the charger only spends energy":
```
t= 305.2 WAIT:60  [routine] waiting away from the charger only spends energy; neural score +10.74, chosen over CHARGE:100 by +0.10
t= 425.2 WAIT:60  [routine] waiting away from the charger only spends energy; neural score +11.99, chosen over PICKUP:A by +0.01
t= 485.2 PICKUP:A [routine] A has 3 units waiting; neural score +12.04, chosen over WAIT:60 by +0.02
```
It delivered 42 units (balanced, same seed: 50). The margins are tiny (0.01–0.10), so the worn condition input tips close decisions toward WAIT. The rules allow it (tier 0).

**`heavy_load` (ns −14 vs symbolic-only).** Also on seed 0, the model chose **WAIT:60 over DELIVER 14 times** while carrying cargo, with margins of 0.02–1.05, while lines were blocked. It lost 26 units. Inputs such as `any_other_blocked` are ~10σ outside the training distribution (file 10 §5.2).

**Common root cause:** the same rule gap in both cases. **WAIT away from the charger is unconstrained in tier 0**, and when the network is uncertain or out of distribution it drifts toward it. Fixing that one rule (file 10 §12 #1) is the cheapest likely improvement, and it can be verified with exactly these two scenarios.

**`low_battery_start` (symbolic-only better than ns).** Both avoid the rule policy's 51-minute charge. The network's learned preference for short or no charging is good, but its routing within tier 0 is slightly worse than "cheapest energy per unit" here.

### 4.4 Generalisation

On unseen scenarios the neuro-symbolic model stays **safe** (0 violations), is **best** in `aged_battery`, `heavy_parts` and `small_buffers` (by small or non-significant margins), and is **clearly worst among the learned/symbolic policies** in `heavy_load`. The hard rules generalise perfectly because they use the scenario's own robot model. The learned ranking does not.

### 4.5 How reliable are these numbers?

- 10 seeds per cell. Per-scenario CIs are ±3–4 points, so differences below about 2 points are mostly not resolvable.
- A 1-hour shift. Energy strategy barely matters (the battery rarely goes below 50 % in most scenarios), and end-of-shift effects are a noticeable fraction of the score.
- One trained model per method, one training seed. The variance between training runs is unknown.

---

## 5. Recommended evaluation protocol for the thesis

1. **More seeds:** 30–50 per scenario (about 5 minutes in total at 0.1 s per episode).
2. **Paired statistics on differences:** mean difference with a bootstrap CI, a paired t-test or Wilcoxon signed-rank, an effect size (Cohen's d_z), and **Holm–Bonferroni** correction across scenarios.
3. **Training-seed variance:** retrain `ns` and `ppo` 3–5 times each and report across-run spread.
4. **Equal-budget learning curves** (score vs simulated actions) for `ns` training and PPO.
5. **Longer shifts** (`--shift-s 28800`, 8 h), so energy and charging strategy genuinely matter.
6. **Fix the end-of-shift effect** in the score (count leftover units, or evaluate a steady-state window).
7. **Report the Pareto view**, not only the scalar score: throughput vs energy vs safety (violations, minimum battery) as a scatter per scenario. Resource efficiency is a trade-off.
8. **Add baselines:** the fallback rules (file 09 §3.4), MCTS with the true simulator (an upper-bound reference), and shielded PPO (file 12 §9).
9. **Gazebo confirmation:** run a subset (e.g. 3 scenarios × 3 seeds × 3 policies, 30 min each) headless in Gazebo with `run.sh … --headless` and compare to the fast-simulator numbers.
10. **Fix `decision_ms`** in `evaluate.py` (always collect latency, not only when tracing).

---

## 6. Key concepts of this section

- Monte-Carlo evaluation over random seeds; episode-level metrics
- Paired experimental design (common random numbers) and paired analysis
- Bootstrap percentile confidence intervals
- Wilcoxon signed-rank test; multiple-comparison correction (Holm)
- Effect size vs statistical significance
- Ablation analysis; generalisation to held-out scenarios
- Multi-objective (Pareto) reporting
- Reproducibility: seeds, training-run variance

---

## 7. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Paired analysis in `evaluate.py`** | The tool's CIs are per policy | Add `--baseline ns`: print per-scenario mean diff, CI and p (`scipy.stats.wilcoxon`, `ttest_rel`), Holm-corrected |
| **Statistical reporting library** | Less hand-written statistics | `rliable` (Agarwal et al. 2021: interquartile mean, performance profiles, stratified bootstrap across tasks). It is designed for exactly "few runs, many tasks" RL evaluation |
| **Parallel evaluation** | Scales to 50 seeds × 12 scenarios × many policies | `multiprocessing.Pool` over (policy, scenario, seed) |
| **Result dashboards / figures** | Thesis figures from CSV | `pandas` + `seaborn`/`matplotlib` notebooks (already in `requirements.txt`), or MLflow/W&B tables |
| **Record full traces for every episode** | Failure analysis needed manual tracing | Save decisions (action, tier, top-2 scores, margin) to Parquet per episode; query "decisions with margin < 0.1 that chose WAIT away from the charger" |
| **Scenario-distribution evaluation** | 12 hand-made points in parameter space | Sample 200 random parameter combinations (Latin hypercube over rates, masses, buffers, battery, ambient) and report performance as a function of utilisation |
