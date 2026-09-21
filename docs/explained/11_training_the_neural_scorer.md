# 11 — Training the neural scorer (no human labels)

File: `tools/ai/train_ns.py` (331 lines).
Config: `params.yaml → mission.neurosymbolic.training`.
Output: `src/robofetch_ai/robofetch_ai/models/ns_scorer.pt` and `tools/ai/results/train_ns_<time>.json`.

```bash
venv/bin/python -u tools/ai/train_ns.py                       # full: 3 rounds, ~40 min on CPU
venv/bin/python tools/ai/train_ns.py --quick                  # smoke test
venv/bin/python tools/ai/train_ns.py --iterations 2 --rollout-policy rule
```

---

## 1. The core problem: where do training targets come from?

Supervised learning needs labels ("in this state, PICKUP:B is best"). Nobody has such labels for this factory, and asking an expert for thousands of them is infeasible and would only copy the expert's biases.

**Solution used here: measure the value of every candidate action by simulation.** The fast simulator can be cloned, so for each decision you can try each option and see what happens. This is known as:
- **Monte-Carlo rollout evaluation** (estimate a value by sampling futures),
- the **rollout algorithm** from approximate dynamic programming (Bertsekas): evaluate each action by *taking it and then following a base policy*,
- **approximate policy iteration** when repeated with the improved policy as the new base.

The network then learns to **generalise** these measured values to states it was never simulated in, so at decision time **no simulation is needed** (1 ms instead of seconds).

---

## 2. Training hyperparameters

```yaml
training:
  episodes: 80                # shifts walked per round (round-robin over scenarios)
  rollout_horizon_s: 900.0    # look-ahead length after each candidate action
  rollout_samples: 3          # different random futures averaged per candidate
  discount_per_100s: 0.97     # future value discount
  exploration: 0.25           # fraction of random choices while walking
  epochs: 40
  batch_size: 256             # NOTE: not used by the ranking trainer (see §6.4)
  learning_rate: 0.001
  validation_fraction: 0.2
  iterations: 3               # policy-iteration rounds
  seed: 12345
```

---

## 3. The algorithm end to end

```
reference ← ns_symbolic_only (or rule, --rollout-policy)
for round = 1..iterations:
    DATA ← collect(scenarios, episodes, reference)            §4
    model ← new ActionScorer(64, 64)
    train_ranking(model, DATA)                                 §6
    validation_score ← mean score of NeuroSymbolicPolicy(full, model) over
                       4 unseen seeds (1000..1003) × each scenario       §7
    if validation_score ≥ best: save model (with metrics)
    reference ← NeuroSymbolicPolicy(full, model)               ← policy iteration
write tools/ai/results/train_ns_<time>.json
```

Each round **trains a fresh network from scratch** on that round's data. Data is not accumulated across rounds.

---

## 4. Data collection (`collect`)

### 4.1 Pseudocode

```python
for episode in range(episodes):
    scenario = scenarios[episode % len(scenarios)]            # round-robin
    sim = FactorySim(scenario, seed + episode)                # seed = 12345 + round_no + episode
    while not sim.done:
        state, legal = sim.state(), sim.legal_actions()
        candidates = top-tier allowed actions (SymbolicLayer)  # only what the model will ever choose between
        if len(candidates) < 2:                                # nothing to learn
            sim.step(reference.decide(...)); continue
        for a in candidates:
            value[a] = rollout_value(sim, a, horizon=900, gamma=0.97, reference, samples=3,
                                     seed = seed + 1000 * point_id)
            estimate = symbolic_estimate(state, a)
            rows.append(action_features(state, a)); targets.append(value[a] - estimate)
            meta.append((scenario, str(a), point_id))
        point_id += 1
        chosen = random.choice(candidates) if rng.random() < 0.25 else reference.decide(...)
        sim.step(chosen)
```

### 4.2 The rollout value

```python
def _one_rollout(sim, action, horizon_s, gamma_per_100s, policy, rng=None):
    clone = copy.deepcopy(sim)
    if rng is not None:                                        # a different possible future
        for sid, section in clone.sections.items():
            section.rng = random.Random(f"{rng}:{sid}:{clone.time_s}")
    start = clone.time_s
    outcome = clone.step(action)
    total = clone.reward(outcome)                              # the action itself: undiscounted
    while not clone.done and clone.time_s - start < horizon_s:
        nxt = policy.decide(clone.state(), clone.legal_actions(), clone).action
        outcome = clone.step(nxt)
        total += gamma_per_100s ** ((clone.time_s - start) / 100.0) * clone.reward(outcome)
    return total
```

$$Q^{\pi_{ref}}_{H}(s, a) \approx r(s, a) + \sum_{k \ge 1,\ t_k - t_0 < H} \gamma_{100}^{(t_k - t_0)/100}\, r_k$$

where $t_k$ is the sim time **at the end** of the k-th follow-up action.

**Discount in time, not per step (the SMDP treatment).** With $\gamma_{100} = 0.97$:

| Elapsed | Weight |
|---|---|
| 100 s | 0.970 |
| 300 s | 0.913 |
| 600 s | 0.833 |
| 900 s | 0.760 |
| 1 h | 0.334 (never reached: horizon 900 s) |

A 60 s WAIT and a 50-minute CHARGE are discounted by the time they really take. That is the correct treatment of variable-duration actions.

**Worked example (hypothetical numbers to show the arithmetic).** Candidate DELIVER with 3 units, estimate +2.9:
- the action: +3 delivered − 0.5 × 0.19 Wh = +2.905
- follow-ups by the reference policy over 900 s deliver 2, 4 and 3 units at t = 250, 540 and 860 s, with small energy costs: 0.927 × 1.9 + 0.848 × 3.9 + 0.771 × 2.9 ≈ 1.76 + 3.31 + 2.24 = 7.31
- $Q \approx 10.2$, and the stored target is $Q - \text{estimate} = 10.2 - 2.9 = 7.3$

This is why the corrections in file 10 §6.3 are around +8 to +14. They contain the **whole discounted future of 15 minutes**, not a small adjustment.

### 4.3 Several futures and common random numbers

Production and faults are random, so **one rollout is a noisy measurement** of an action's value. With `rollout_samples = 3`, three rollouts are averaged, each with section RNGs reseeded from `seed + i`.

The detail that matters is that `seed = seed + 1000 * point_id` is **the same for all candidates at one decision point**. Candidate PICKUP:A and candidate DELIVER are evaluated against the **same three random futures** (the same reseeding at the same `time_s`).

**Concept: common random numbers (CRN).** When comparing alternatives by simulation, using the same random draws for each alternative makes the *difference* between them much less noisy than the individual values, because shared luck cancels out. Since the ranking loss (§6) only cares about differences within a decision, CRN is exactly the right variance-reduction trick. (The futures still diverge after the actions differ, because draws are consumed in different orders, so CRN is partial.)

**If `rollout_samples = 1`** (e.g. `--quick`), the `rng` is `None`, the clone keeps the simulator's **actual** RNG state, and the rollout sees *the true future* of that episode. The label then contains information no real decision could have (a mild leak). The full config uses 3 samples, so the committed model is not affected.

### 4.4 Which states and which actions get labelled

- **Which states:** the walk follows the **reference policy** with 25 % random exploration. In round 1 that is the symbolic-only policy. In rounds 2–3 it is the previous round's model. The docstring's reasoning: *"A network trained on someone else's states is asked at run time about situations it never saw."* That idea is **DAgger** (Dataset Aggregation, Ross et al. 2011): train on the state distribution the learner itself induces, to avoid compounding errors. Here it is applied in a simple form. No data is aggregated across rounds; each round uses its own policy's states.
- **Which actions:** only the **allowed top-tier candidates**, because the full model only ever chooses among those. States with a single candidate are skipped (the rules decide alone).
- **Exploration** picks a random candidate *among the allowed top tier*, so it is safe exploration and the walked states stay realistic.

Consequence: the network has **never been trained on forbidden or lower-tier actions**. `ns_neural_only`, which scores all legal actions, uses the network far outside its training data. Its failures (20/20 violations in low battery, HANDOVER WP5) say little about how a network *trained without rules* would behave.

### 4.5 Scale

Committed model (from `train_ns_20260916_165028.json`):

| Round | Rollout policy | Samples | Collect time | Val top-1 | Train top-1 | Validation score |
|---|---|---|---|---|---|---|
| **1** | `ns_symbolic_only` | 13 536 | 596 s | **35.0 %** | 41.5 % | **59.68** ← saved |
| 2 | `ns` (round-1 model) | 13 973 | 791 s | 35.3 % | 47.1 % | 58.05 |
| 3 | `ns` (round-2 model) | 12 177 | 583 s | 34.9 % | 46.4 % | 56.10 |

About 13 500 (action, value) rows means about 4 000–5 000 decision points with 2–4 candidates each. Collection dominates the run time: every candidate costs 3 rollouts of up to 900 s of simulation, each involving the reference policy's own rule checks.

---

## 5. What is learnt: the residual

```python
targets.append(value - estimate)          # the network learns the CORRECTION
```

At decision time: `score = symbolic_estimate + network(features)`. See file 10 §6 for why, and for real numbers.

---

## 6. The ranking loss (`train_ranking`)

### 6.1 Why not plain regression?

HANDOVER WP5 recorded the reasoning: *"The look-ahead value carries ±2.5 units of noise while two candidates typically differ by < 1 unit, so regressing values spent the network on noise."* With mean-squared error, the network tries to predict the absolute value of each candidate. Most of that value (what happens in the next 15 minutes regardless of this choice) is shared by all candidates and dominated by noise. **The decision only needs the ordering.** Switching the loss took the validation score from 56.8 to 59.7 (WP5).

### 6.2 Concept: listwise learning to rank with softmax cross-entropy

For one decision point with candidates $a_1..a_n$, predicted scores $s_i$ (estimate + correction) and measured values $v_i$:

$$P_\theta(a_i) = \frac{e^{s_i / T}}{\sum_j e^{s_j / T}}, \qquad y = \arg\max_i v_i$$

$$\mathcal{L}_{rank} = -\log P_\theta(a_y)$$

This is **cross-entropy with the look-ahead's best action as the class label**. It is the "top-1" variant of listwise ranking (related to ListNet). The loss is low when the best candidate gets most of the probability mass. Only **differences** between scores matter, so a shared offset in all values is ignored and the shared noise cancels.

`temperature` T = 1.0 here.

### 6.3 Plus a small value term

```python
correction = model(x[idx])
scores = (correction + estimates[idx]) / temperature
target = torch.argmax(y[idx] + estimates[idx])        # = argmax of the measured value
loss = cross_entropy(scores, target) + 0.2 * mse_loss(correction, y[idx])
```

$$\mathcal{L} = \mathcal{L}_{rank} + 0.2 \cdot \frac{1}{n}\sum_i (c_i - (v_i - \hat{V}_{sym,i}))^2$$

The MSE term keeps the scores **on a meaningful scale** (discounted units), so the explanation "neural score +16.56, chosen by +0.60" is interpretable. Without it, cross-entropy is shift-invariant and scores could drift to any offset.

### 6.4 Optimisation details

- **Grouping:** rows are grouped by `point_id` (one decision point = one group).
- **Train/validation split by decision point**, not by row: 20 % of decision points are held out. Splitting by row would leak, because candidates of the same decision share the state and would appear in both sets.
- **Normalisation:** `model.set_normalisation(x[train rows])`, from training rows only.
- **Optimiser:** Adam, lr 1e-3.
- **One update per decision point.** Each group is a mini-batch of 2–4 rows. `batch_size: 256` in the config is **not used** by this function; it belonged to the old regression trainer `train()`, which is still in the file but never called.
- 40 epochs; each epoch shuffles decision points with `Random(seed + epoch)`.
- Reported every 5 epochs: *"picks the look-ahead's best action X % of the time"*.

**Concept: Adam.** Adam keeps running averages of the gradient (momentum) and its square (per-parameter scale), which gives adaptive step sizes that work well without tuning for small networks like this.

### 6.5 The metric: top-1 agreement

```python
hits += argmax(model(x) + estimates) == argmax(y + estimates)
```

This is the fraction of held-out decisions where the model's top choice equals the look-ahead's best. With 2–4 candidates, random guessing gives 25–50 % (1/n). **35 % validation top-1 is close to chance level.** Train top-1 (41–47 %) is higher, which is mild overfitting.

How can the model still perform well (file 13)? Three reasons:
1. The rules already restrict choices to good, safe, priority-correct options, so a "wrong" pick among them is often nearly as good (margins are small).
2. The look-ahead "best" is itself noisy (3 samples), so the true agreement ceiling is well below 100 %.
3. The network learned some **strong, consistent patterns** (e.g. never charge to 70/100 % early when buffers are filling), which matter a lot for the score even if fine-grained rankings are noisy.

The last point is a hypothesis consistent with the traces, not a measured decomposition. A thesis should test it, e.g. by measuring agreement separately per action type.

---

## 7. Round evaluation and model selection (`evaluate_model`)

```python
for scenario in scenarios:                   # the 6 training scenarios
    for seed in range(1000, 1004):           # 4 seeds never used in collection or evaluation
        score = run_episode(NeuroSymbolicPolicy(full, scorer), scenario, seed)
return mean(score)                            # 24 episodes
```

The round with the best validation score is saved. This was added because *"the last round was being saved even when worse"* (WP5 fix 2).

**Statistical caveat:** 24 episodes give a mean with a standard error of roughly ±1–2 points (scores differ by scenario from about 30 to 95). The three rounds differ by 1.6 and 3.6 points. Picking the maximum of noisy estimates has **selection bias** (the winner's curse): the saved round's true score is probably lower than 59.68. The differences between rounds may not be real. A fixed, larger validation set, or a paired comparison of rounds on the same seeds with a CI, would settle it.

**Seed hygiene is good:** collection uses seeds 12346 + episode and upward, validation uses 1000–1003, and final evaluation (`evaluate.py`) uses 0–19. No overlap.

---

## 8. Policy iteration: why it did not help (yet)

**Theory.** For exact evaluation, the rollout policy "take the action with the best Q under the base policy" is **at least as good as the base policy** (policy improvement theorem). Repeating the step with the improved policy converges towards optimal. That is **policy iteration**.

**Practice here:** rounds 2 and 3 were worse (58.05, 56.10). Plausible reasons, all consistent with the data:
1. **Approximation error compounds.** The round-2 reference is not the exact rollout policy; it is a 35 %-agreement *approximation* of it. Its look-ahead values are measured with a weaker base policy than intended.
2. **Noisy targets** (3 samples) and a **truncated horizon** (900 s) make improvement steps small compared to noise.
3. **No data aggregation.** Each round discards previous data, so the network sees fewer and different states.
4. **Validation noise** (§7) may hide small real improvements or invent differences.

HANDOVER also records that DAgger-style state walking alone made the model slightly worse (56.8 vs 58.3). It was kept because it is methodologically right. The ranking loss was the change that helped.

---

## 9. Reproducing and inspecting

```bash
venv/bin/python -u tools/ai/train_ns.py | tee logs/train_ns.log
python3 -c "import json; d=json.load(open('tools/ai/results/<file>.json')); print([ {k:r[k] for k in ('round','samples','val_top1','validation_score')} for r in d['rounds']])"
venv/bin/python -c "from robofetch_ai.policies.neural import ActionScorer; m=ActionScorer.load(); print(m.info)"
```

The JSON report contains every round's metrics, the training config and the scenario list, plus `history` (per-epoch loss and top-1) **of the last round only**. The `history` variable is overwritten each round.

---

## 10. Key concepts of this section

- Monte-Carlo rollout evaluation; truncated horizon; time-based discounting for SMDPs
- The rollout algorithm and approximate policy iteration (Bertsekas)
- Policy improvement theorem
- Common random numbers for variance reduction in simulation comparisons
- On-policy state distributions, covariate shift, DAgger
- Safe exploration inside a constraint set
- Residual targets
- Learning to rank: pointwise (MSE) vs listwise (softmax cross-entropy top-1)
- Grouped train/validation splits to prevent leakage
- Adam optimiser, epochs, overfitting signals (train vs validation gap)
- Model selection bias with noisy validation

---

## 11. Improvements and technologies for this section

| # | Idea | Why | How |
|---|---|---|---|
| 1 | **Aggregate data across rounds** (real DAgger) | Each round throws away previous data | Keep a replay dataset; round k trains on rounds 1..k, optionally re-labelling old states with the new reference |
| 2 | **More samples where it matters** | Top-1 near chance; labels noisy | Adaptive sampling: run 3 futures, and add more only for decisions where the top-2 values are within the noise (a racing / successive-halving scheme) |
| 3 | **Pairwise loss with margin and label noise awareness** | The argmax label is wrong whenever noise flips the order | Use the *probability* that a beats b from the samples (mean/σ) as a soft target: `BCE(sigmoid(s_a - s_b), P(v_a > v_b))` (RankNet-style) |
| 4 | **Paired, larger validation** | Round selection is within noise | Validate all rounds on the same 60+ episodes; select only if the paired CI of the difference excludes 0; otherwise keep the simpler model |
| 5 | **Parallel rollouts** | 10–13 min per round, single core | `concurrent.futures.ProcessPoolExecutor` per decision point or per episode; `ray` for a cluster. Collection is embarrassingly parallel |
| 6 | **Replace deep copy** | `copy.deepcopy(sim)` is the main cost | A lightweight `FactorySim.clone()` that copies only numbers and RNG states |
| 7 | **Train on all 12 scenarios + randomised parameters** | OOD failures | Sample rates, masses, buffers, battery, ambient from ranges each episode (domain randomisation); keep 2–3 scenarios fully held out for testing generalisation |
| 8 | **Hyperparameter search** | Horizon, discount, samples, exploration, value weight, network size were set by hand | `Optuna` with the paired validation score as objective (with a budget: each trial is a collection + training run) |
| 9 | **Experiment tracking** | Runs are JSON files named by time | `MLflow` or `Weights & Biases`: log config, metrics per epoch/round, the model artefact and the git commit, for thesis reproducibility |
| 10 | **Online search instead of (or on top of) learning** | Rollouts are already the "teacher"; they could be used at decision time too | MCTS / rollout policy at decision time with a time budget (e.g. 200 ms live), using the network as prior and value (AlphaZero-style), still behind the symbolic shield |
| 11 | **Distillation from a stronger teacher** | The teacher is only "take action, then symbolic policy" | Use an expensive teacher (MCTS with many simulations, or a longer horizon with more samples) offline to label states, then distil into the small scorer |
| 12 | **Clean up** | Unused `train()`, unused `batch_size`, last-round-only `history` | Remove dead code; store history per round |
