# 09 — The policy interface, the rule-based reference, and the fallback rules

Files:
- `src/robofetch_ai/robofetch_ai/policies/base.py`: `Policy`, `Decision`
- `src/robofetch_ai/robofetch_ai/policies/rule_based.py`: `RuleBasedPolicy` (116 lines)
- `src/robofetch_core/robofetch_core/fallback_policy.py`: `decide()` (41 lines)
- Tests: `src/robofetch_ai/test/test_rule_policy.py`

These are the **baselines**. A learned model is only interesting if it beats a sensible hand-written policy. The fallback is also what keeps the robot working when the AI is down.

---

## 1. One interface for every decision model

```python
@dataclass
class Decision:
    action: object                              # mission_plan.Action
    explanation: str = ""                       # why, in words
    scores: dict = field(default_factory=dict)  # score per candidate, for analysis
    latency_ms: float = 0.0

class Policy:
    name = "policy"
    def reset(self): ...                                     # called at the start of every shift
    def decide(self, state, legal_actions, sim=None): ...    # -> Decision
```

**Why this matters:**
- **Strategy pattern.** `run_episode`, `evaluate.py`, `train_ns.py` and `service.py` work with *any* policy unchanged.
- **The explanation is part of the contract**, not decoration. The dashboard shows it, and the thesis uses it as evidence of explainability.
- **`sim` is the world object** (`FactorySim` offline, `LiveState` live). Policies read `sim.p` (robot parameters), `sim.matrix` (distances) and `sim.charge_targets()`. PPO reads the observation through it.

Implementations:

| Class | `name` | File |
|---|---|---|
| `RuleBasedPolicy` | `rule` | `rule_based.py` |
| `NeuroSymbolicPolicy(mode="full")` | `ns` | `neurosymbolic.py` (file 10) |
| `NeuroSymbolicPolicy(mode="neural")` | `ns_neural_only` | ablation |
| `NeuroSymbolicPolicy(mode="symbolic")` | `ns_symbolic_only` | ablation |
| `PPOPolicy` | `ppo` / `ppo_unmasked` | `rl_ppo.py` (file 12) |
| `fallback_policy.decide` | `fallback` (not a `Policy` subclass) | `robofetch_core` |

---

## 2. The rule-based reference policy

### 2.1 The five operator rules (docstring)

```
1. keep the robot able to get home    charge when the battery would not cover "reach the charger + reserve"
2. never let a line stand still       serve BLOCKED sections first, then the one that will block soonest
3. do not drive half empty            fill the load on the way if another section is close and has units
4. deliver what you carry             when full, or when nothing is worth collecting
5. wait at the charger, not the aisle waiting anywhere else only wastes energy
```

Parameters: `urgency_horizon_s = 180` (a line blocking within 3 min is urgent) and `fill_threshold = 0.35` (below 35 % fill a trip is not worth it unless urgent).

### 2.2 The code, rule by rule

**Pre-computation:**
```python
urgent = any(s["status"] == "BLOCKED" or s["time_to_full_s"] <= 180 for s in sections)
```

**Rule 1: be able to get home** (only if some CHARGE is legal)
```python
reserve_needed = battery_percent_for(p, trip_energy_wh(p, distance_to_charger_m, payload, T, condition))
if battery <= reserve_needed + reserve_percent:
    if DELIVER is legal and distance_to_delivery <= distance_to_charger:
        return DELIVER   "battery near reserve and the delivery point is on the way"
    return CHARGE:40 if urgent else CHARGE:100
```

**Rules 2/3: who needs the robot most**
```python
worth = [pickups where fill >= 0.35 or BLOCKED or time_to_full <= 180]
urgency(a) = (2 if BLOCKED else 1 if time_to_full <= 180 else 0,          # tier
              units_that_fit × unit_mass / max(1, distance))              # kg per metre
if worth: return max(worth, key=urgency)
```

The tuple is compared lexicographically: first by tier, then by **kilograms collectable per metre driven**. That tie-breaker is a greedy efficiency heuristic.

**Rule 4:** `if DELIVER legal: return DELIVER` ("carrying N units and nothing is urgent").

**Rule 5:**
```python
if location != charger and CHARGE legal:
    return CHARGE:<highest>     "nothing to do - waiting at the charger is free (and tops the battery up)"
return WAIT:60                  "nothing ready to collect; waiting at the charger"
```

### 2.3 Worked example 1: the first minutes of balanced, seed 0

Buffers start at A 1/8, B 2/10, C 0/3. No section reaches 35 % fill and none is urgent:

```
t=   0.0 WAIT:60      nothing ready to collect; waiting at the charger
t=  60.0 WAIT:60      ...
t= 240.0 WAIT:60      ...
t= 300.0 PICKUP:B     section B has the best load per metre (4/10 units, 15.2 m away)
t= 368.9 PICKUP:A     section A has the best load per metre (3/8 units, 18.3 m away)
t= 445.8 DELIVER      carrying 7 units and nothing is urgent
t= 501.5 CHARGE:100   nothing to do - waiting at the charger is free (and tops the battery up)
t= 642.5 WAIT:60      nothing ready to collect; waiting at the charger
```

It batches (7 units in one delivery), waits for free at the charger, and uses little energy. Whole shift: **43 delivered, 0 lost, 4.12 Wh, score 40.94**.

### 2.4 Worked example 2: why it collapses in `low_battery_start`

The robot starts at 30 % with buffers half full (A 4, B 5, C 1):

```
t=    0.0 PICKUP:A     batt=29.4  section A has the best load per metre (4/8 units, 9.0 m away)
t=   53.2 PICKUP:B     batt=28.2  section B has the best load per metre (5/10 units, 18.3 m away)
t=  130.1 DELIVER      batt=27.3  carrying 10 units and nothing is urgent
t=  186.4 CHARGE:100   batt=100.0 nothing to do - waiting at the charger is free (and tops the battery up)
t= 3239.4 PICKUP:A     batt=99.4  section A is BLOCKED (8/8 units, 9.0 m away)
t= 3292.6 PICKUP:B     batt=98.0  section B is BLOCKED (10/10 units, 18.3 m away)
```

**The CHARGE:100 at t = 186 s takes 3053 s (51 minutes).** While charging, all three lines fill up and block. Result for this seed: score −2.68, **25.8 units lost**. Over 10 seeds (fresh evaluation, file 13): score **−3.62**, lost **26.6**.

Two design flaws combine here:
1. **Rule 5 always picks the highest target** (`_charge(legal, urgent=False)`), even when nothing is urgent *right now* but lines will need the robot soon.
2. **CHARGE is not interruptible.** Once started it runs to the target, whatever happens to the factory. A human would glance at the lines every few minutes.

This is exactly the gap the learned models exploit (the neuro-symbolic model scores 49.7 in the same scenario, file 13).

### 2.5 Worked example 3: `heavy_load` (score −22.4)

At double rates with smaller buffers the rule policy starts with two WAITs (nothing ≥ 35 % yet) and then does many small collection trips while lines keep blocking. It loses about **57 units per shift**. Its strict "fill ≥ 35 %" and "load per metre" heuristics are tuned for nominal rates and do not adapt. The simple fallback rules (§3) score **+24.8** in the same scenario, because they always serve the section that fills soonest.

### 2.6 Weak spots in the implementation

1. **Dead code:** `battery_needed()` is never called. Its helper `_home_distance(state, location)` **ignores its `location` argument** and returns `max(distance_to_charger, distance_to_delivery)` from the *current* position, which is wrong for "home after the goal". If someone starts using it, the bug becomes live.
2. **No feasibility check for pickups:** rules 2/3 never ask whether the robot can still get home after the pickup trip. Rule 1 only looks at the *current* distance to the charger. With 15 % reserve and short trips this rarely matters (0 violations in all 120 evaluation episodes), but it is not a guarantee.
3. **`urgency_horizon_s` does not consider travel time.** A line 18 m away (about 47 s of driving plus 30 s of loading) that blocks in 60 s is "urgent" but cannot be saved. The symbolic layer uses `max(horizon, travel_s)`.
4. **`latency_ms` is never set** (always 0), so evaluation reports 0 ms for this policy.

---

## 3. The fallback policy (no AI)

### 3.1 Purpose and placement

The docstring says: *"Deliberately tiny, dependency-free and dull: the robot must keep working safely even with no AI at all, and this must not itself be able to fail. It lives in robofetch_core (not robofetch_ai) precisely so that the fallback does not depend on the thing that failed."*

It is used by the executor when:
- `model:=fallback` (chosen on purpose, the "no AI" mode in `run.sh`), or
- the HTTP call to the decision service fails (connection refused, timeout, error JSON, unparseable answer).

### 3.2 The four rules

```python
home = battery_percent_for(p, trip_energy_wh(p, distance_to_charger, payload, T, condition))
1. if battery <= home + reserve + 2.0:
       return CHARGE:<max target = 100>       "fallback: battery X % barely covers getting home"
2. candidates = sections with units_that_fit > 0
   if candidates:
       return PICKUP of max(candidates, key = (BLOCKED ? 1 : 0, -time_to_full))
                                               "fallback: section B fills in 928 s"
3. if cargo_units > 0: return DELIVER           "fallback: carrying N units, nothing to collect"
4. return WAIT:60                               "fallback: nothing to do"
```

### 3.3 Rule-based vs fallback, side by side

| Aspect | `RuleBasedPolicy` | `fallback_policy` |
|---|---|---|
| Charging threshold | battery ≤ home + reserve | battery ≤ home + reserve + 2 |
| Charge target | 40 if any line urgent, else 100 | always the highest (100) |
| Deliver-first when low | yes, if delivery is on the way | no |
| Pickup filter | fill ≥ 35 % or urgent | **any** section with ≥ 1 unit that fits |
| Pickup ordering | tier, then kg per metre | BLOCKED first, then soonest to fill |
| Idle behaviour | go to charger (CHARGE) or WAIT there | WAIT **wherever it is** |
| Legal-action aware | yes (given `legal_actions`) | **no** (builds actions itself) |

### 3.4 How good is "no AI"? (fresh measurement)

The fallback was never evaluated in the fast simulator, so I wrapped it as a `Policy` and ran 10 seeds per scenario (same seeds 0–9 as the fresh evaluation in file 13):

| Scenario | Fallback score | Delivered | Lost | Energy Wh | Min battery % | Rule score (for comparison) |
|---|---|---|---|---|---|---|
| aged_battery | 36.61 | 41.6 | 0.00 | 9.99 | 16.7 | 44.72 |
| balanced | 42.95 | 48.5 | 0.00 | 11.09 | 49.6 | 45.18 |
| fault_burst | 28.98 | 33.8 | 0.00 | 9.64 | 56.2 | 30.15 |
| heavy_load | **24.79** | 57.3 | 28.81 | 7.40 | 16.4 | −22.41 |
| heavy_parts | 45.24 | 50.9 | 0.00 | 11.32 | 48.5 | 44.42 |
| high_demand | **95.16** | 101.1 | 0.00 | 11.87 | 46.0 | 91.67 |
| hot_factory | 42.77 | 48.5 | 0.00 | 11.45 | 47.9 | 45.07 |
| low_battery_start | −4.88 | 16.7 | 20.09 | 3.00 | 16.4 | −3.62 |
| one_hot_section | 69.42 | 75.3 | 0.00 | 11.77 | 46.5 | 72.64 |
| section_breakdown | 29.51 | 34.3 | 0.00 | 9.59 | 56.4 | 29.14 |
| small_buffers | 42.84 | 48.9 | 0.52 | 11.09 | 49.6 | 43.68 |
| worn_robot | 42.54 | 48.5 | 0.00 | 11.93 | 45.8 | 46.11 |

No safety violations occurred in any episode. **Observations:**
- The "dull" fallback is **within 2 % of the neuro-symbolic model in `high_demand`** (95.2 vs 96.7) and **far better than the rule policy in `heavy_load`**. Its greedy "serve whatever fills soonest, even 1 unit" behaviour is a strong heuristic when demand is high.
- It uses about **2–2.5× the energy of the rule policy** (1-unit trips).
- It fails in `low_battery_start` for the same reason as the rule policy: a non-interruptible CHARGE:100 when the battery gets low.
- **For the thesis this is a useful extra baseline.** It shows how much of the learned models' advantage comes from "just serve the soonest-full line" versus real judgement.

(The wrapper replaced an illegal fallback action with WAIT. In practice the fallback only proposes legal actions, except when the battery rule fires while already at 99 %+, which does not happen here.)

---

## 4. Key concepts of this section

- **Strategy pattern / common interface** for interchangeable decision models
- **Heuristic (rule-based) scheduling**: priority dispatching rules like "earliest due date" (soonest to block) and "shortest processing time"
- **Lexicographic ordering** of priorities (tier, then efficiency)
- **Myopic vs look-ahead decisions**: all rules here look one action ahead
- **Graceful degradation** and **fault isolation** by package boundaries
- **Baselines** in experimental design: a strong simple baseline is the honest comparison

---

## 5. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Add the fallback to `evaluate.py`** | It is the real no-AI mode and a surprisingly strong baseline | A `FallbackPolicy(Policy)` wrapper like the one used above; add `"fallback"` to `POLICIES` |
| **Partial charging in both rule sets** | CHARGE:100 while lines fill is the single biggest loss in `low_battery_start` | Charge to the lowest target that covers the next predicted hour of work, or to 40 % when any line blocks within `charge_time(40)` |
| **Re-evaluate during long actions** | A 51-minute charge cannot react | Split CHARGE into slices (e.g. `CHARGE_SLICE:300s` action) or let the executor re-ask every N s with "continue" as an option |
| **Feasibility check in rules 2/3** | Pickup trips are not checked against the reserve | Reuse `SymbolicLayer.check` hard rules as a filter before choosing |
| **Remove or fix dead code** | `battery_needed` + buggy `_home_distance` | Delete, or implement with `matrix[destination][charger]` |
| **Classic dispatching / OR baselines** | The thesis compares against one hand-made rule set; operations research has strong, well-known alternatives | (a) **Rolling-horizon MILP** with `OR-Tools CP-SAT` or `Pyomo`: every decision, solve a short-horizon routing + charging schedule on expected production; (b) **Vehicle Routing Problem with time windows** where the "time window" is time-to-full |
| **Monte-Carlo tree search (MCTS) baseline** | The simulator is cheap and cloneable, so planning at decision time is possible | UCT over the 8 macro-actions with depth ~6–10 and the rule policy as rollout policy; it is an *online* planner with no training at all, a strong reference for "how good is possible" |
| **Tune the rule parameters** | 180 s and 0.35 were chosen by hand | Bayesian optimisation (`Optuna`) of `urgency_horizon_s`, `fill_threshold`, charge thresholds over training seeds; report on separate seeds |
