# 10 — The neuro-symbolic decision model (the thesis contribution)

Files:
- `src/robofetch_ai/robofetch_ai/policies/neurosymbolic.py` (137 lines): `NeuroSymbolicPolicy`
- `src/robofetch_ai/robofetch_ai/policies/symbolic.py` (130 lines): `SymbolicLayer`, `Verdict`
- `src/robofetch_ai/robofetch_ai/policies/features.py` (125 lines): `FEATURE_NAMES`, `action_features`, `symbolic_estimate`
- `src/robofetch_ai/robofetch_ai/policies/neural.py` (69 lines): `ActionScorer`
- `src/robofetch_ai/robofetch_ai/models/ns_scorer.pt`: trained weights
- `params.yaml → mission.neurosymbolic`
- Tests: `test_symbolic.py` (11), `test_neurosymbolic.py` (10 + 12 parametrised)

Training is in file 11. Results are in file 13.

---

## 1. What "neuro-symbolic" means, and which kind this is

**Symbolic AI** reasons with explicit, human-readable knowledge: rules, logic, models. It is transparent and gives guarantees, but it is brittle and needs someone to write down every judgement.

**Neural (sub-symbolic) AI** learns a function from data. It is flexible and good at fuzzy trade-offs, but opaque and without guarantees.

**Neuro-symbolic AI** combines the two. There are many ways to do that (Henry Kautz's taxonomy lists several: symbolic systems calling neural subroutines, neural networks with symbolic constraints in the loss, networks that output symbols, and so on).

**This project's architecture is a pipeline: symbolic filtering and prioritisation, then neural ranking.**

```
                    legal actions (from the simulator / live state mask)
                              │
             ┌────────────────▼────────────────┐
             │  SYMBOLIC — HARD RULES (forbid)  │   safety & physics, checked with robot_model
             │  H1 maintenance  H2 payload      │   → forbidden actions removed, with reasons
             │  H3 battery reserve  H4 heat     │   → NOTHING can override this
             └────────────────┬────────────────┘
                              │ allowed actions + verdicts
             ┌────────────────▼────────────────┐
             │  SYMBOLIC — PRIORITY RULES       │   tier 2: unblock a stopped line / battery at reserve
             │  keep only the highest tier      │   tier 1: prevent a stop / robot is full
             └────────────────┬────────────────┘   tier 0: routine
                              │ candidates (same tier)
             ┌────────────────▼────────────────┐
             │  NEURAL SCORER (rank)            │   27 features per candidate → MLP → correction
             │  score = symbolic_estimate + NN  │   pick the highest score
             └────────────────┬────────────────┘
                              ▼
          Decision(action, explanation = tier + rule reason + score + margin + refusals)
```

It is closely related to three established ideas:
- **Shielding / safe action filtering** in safe reinforcement learning. A shield removes unsafe actions before (or after) a learned policy chooses (Alshiekh et al., "Safe Reinforcement Learning via Shielding", 2018).
- **Lexicographic multi-objective decision making.** Priorities are strict: a tier-2 action always beats a tier-1 action, whatever the scores.
- **Learning to rank / residual learning.** The network ranks candidates and learns a *correction* to an analytic estimate.

**The guarantee this structure gives:** *whatever the network outputs, the chosen action satisfies the hard rules as evaluated by the robot model.* The test suite proves this with a sabotaged network (§9). The guarantee is only as good as the model the rules use (file 05) and the rules' coverage (§10).

---

## 2. Configuration

```yaml
mission:
  neurosymbolic:
    urgency_horizon_s: 180.0      # a line that blocks within this is urgent (tier 1)
    safety_margin_percent: 2.0    # extra battery on top of the reserve before an action is allowed
    hidden_sizes: [64, 64]
    training: {...}               # file 11
```

---

## 3. The symbolic layer (`symbolic.py`)

### 3.1 `Verdict`

```python
@dataclass
class Verdict:
    allowed: bool = True
    tier: int = 0
    refusals: list      # why a hard rule forbids it
    reasons: list       # why it is (or is not) urgent
    def why(self): return "; ".join(refusals if not allowed else reasons) or "routine"
```

`SymbolicLayer.check(state, action, p, matrix)` returns one `Verdict`. `evaluate()` returns `{action: Verdict}` for all legal actions.

### 3.2 Hard rules

**H1: maintenance.** A worn-out robot may only charge or wait.
```python
if state["condition_percent"] < p.condition_min_percent (30) and action.kind in (PICKUP, DELIVER):
    forbid: "condition 28 % is below the maintenance limit (30 %)"
```

**H2: payload.** Never plan to carry more than the limit.
```python
if action.kind == PICKUP and sections[target]["units_that_fit"] <= 0:
    forbid: "nothing at B that still fits (4.8 of 5.0 kg carried)"
```
(The legal-action mask already removes such pickups, so in normal operation H2 never fires. It is a second line of defence, e.g. for `ns_neural_only` comparisons or a different mask.)

**H3: battery reserve.** After this action the robot must still reach the charger with reserve + margin.
```python
if dest and action.kind != CHARGE:           # PICKUP or DELIVER (WAIT has no destination)
    legs = [(matrix[location][dest], payload),
            (matrix[dest][CHARGER], payload if PICKUP else 0.0)]
    end, peak = simulate_route(RobotCondition(p, battery, temperature, condition), legs,
                               dwell_s = load_time if PICKUP else unload_time)
    need = reserve_percent + safety_margin     # 15 + 2 = 17 %
    if end.battery_percent < need:
        forbid: "battery would be 16 % back at the charger, below the 17 % reserve"
```
The rule **forward-simulates** the route with the real robot model: temperature-dependent efficiency, wear penalty, payload energy, idle power and the dwell (file 05 §8).

**H4: heat** (same simulation).
```python
    if peak >= p.max_c (70):
        forbid: "motors would reach 72 C (limit 70 C)"
```

**What H3 assumes, precisely:**
- The return leg after a PICKUP uses the payload **before** the pickup, not the heavier load after it. It slightly underestimates the return energy. With 0.00116 Wh/m/kg the error is tiny (for example 4 kg × 15 m = 0.07 Wh = 0.3 %), and the 2 % margin covers it. It is still a known imprecision.
- It checks "go there, then go home". It does not check the *next* intended action, so it is a **one-step look-ahead safety guarantee**. Because it always keeps the way home open, one step is enough to never strand the robot, as long as CHARGE stays possible.
- **WAIT is never checked.** A WAIT away from the charger drains 6 W (0.45 % per minute) without any rule. Combined with §3.4 this is a real gap (see the worked example in §7.3).

### 3.3 Priority rules (only for allowed actions)

| Action | Tier 2 | Tier 1 | Tier 0 (reason text) |
|---|---|---|---|
| `PICKUP:X` | X is **BLOCKED** ("X is BLOCKED and losing production") | X's `time_to_full_s ≤ max(urgency_horizon 180 s, travel time to X)` ("X fills in 150 s (47 s away)") | "X has N units waiting" |
| `DELIVER` | — | robot is **full** (`free_payload_kg ≤ 0.01`) ("the robot is full and cannot collect anything else") | "carrying N units" |
| `CHARGE:p` | battery ≤ reserve + home + margin ("battery 17 % is at the reserve") | — | "top up from 45 %" |
| `WAIT` | — | — | "waiting at the charger is free and tops the battery up" / "waiting away from the charger only spends energy" |

`max(urgency_horizon, travel_s)` means that a far-away section counts as urgent earlier, which is travel-time aware.

```python
@staticmethod
def allowed_by_tier(verdicts):
    allowed = {a: v for a, v in verdicts.items() if v.allowed}
    if not allowed: return {}, None
    top = max(v.tier for v in allowed.values())
    return {a: v for a, v in allowed.items() if v.tier == top}, top
```

Only the **highest tier present** survives. If B is BLOCKED, the network may only choose among tier-2 actions. It cannot decide to deliver first, however high it would score DELIVER.

### 3.4 An inconsistency between H3 and the CHARGE tier

The CHARGE tier-2 test is:
```python
home_percent = battery_percent_for(p, p.idle_power_w * (matrix[location][CHARGER] / p.speed_m_s) / 3600)
tier 2 if battery <= reserve + home_percent + margin
```
It uses **only the idle energy** of the trip home, not the drive energy. H3 uses the full simulated energy of *two* legs plus the dwell. So there is a **dead band**: battery values where H3 already forbids all work but CHARGE is not yet tier 2. Then CHARGE and WAIT are both tier 0, and the **network** decides between them. §7.3 shows a real example where it chooses to WAIT away from the charger.

---

## 4. Features (`features.py`)

### 4.1 One feature vector per *candidate action*, not per state

The docstring says: *"Scoring each action separately (rather than one vector for the whole state) is what lets the neural part stay small and generalise: it learns 'is this action worth it, given what it costs and what it saves', not 'what should I do in this exact factory'."*

This is a **Q-function-like, action-in design**: $f(s, a) \in \mathbb{R}^{27}$, and the network outputs a scalar per $(s, a)$. The same network can score any number of candidates, so the action set could grow without changing the architecture. That is different from PPO's fixed 8-output head.

### 4.2 All 27 features

With `dest` = the action's destination, `d` = distance to it, `E` = trip energy there with the current payload/temperature/condition, and `cost%` = E as battery %:

| # | Name | Definition | Scale |
|---|---|---|---|
| 0 | `is_pickup` | action is PICKUP | 0/1 |
| 1 | `is_deliver` | action is DELIVER | 0/1 |
| 2 | `is_charge` | action is CHARGE | 0/1 |
| 3 | `is_wait` | action is WAIT | 0/1 |
| 4 | `battery` | battery % | /100 |
| 5 | `temperature` | (T − ambient)/(max − ambient) | 0..1 |
| 6 | `condition` | condition % | /100 |
| 7 | `payload_full` | payload / max payload | 0..1 |
| 8 | `cargo_units` | min(1, units/20) | 0..1 |
| 9 | `time_left` | min(1, time left / 3600) | 0..1 |
| 10 | `distance` | min(1, d / 25) | 0..1 |
| 11 | `energy_cost_percent` | min(1, cost% / 20) | 0..1 |
| 12 | `battery_after` | (battery − cost%) / 100 | |
| 13 | `battery_after_home` | (battery − cost% − home%) / 100, home% = trip energy dest→charger | can be < 0 |
| 14 | `target_fill` | PICKUP: target buffer fill; else 0 | 0..1 |
| 15 | `target_units_gain` | PICKUP: min(1, units that fit / 15) | 0..1 |
| 16 | `target_mass_gain` | PICKUP: units × unit mass / max payload | 0..1 |
| 17 | `target_time_to_full` | PICKUP: min(1, ttf/600), inf → 1; else 1 | 0..1 |
| 18 | `target_rate` | PICKUP: actual/nominal rate | 0..1 |
| 19 | `target_health` | PICKUP: health/100 | 0..1 |
| 20 | `target_blocked` | PICKUP: target BLOCKED | 0/1 |
| 21 | `target_fault` | PICKUP: target FAULT | 0/1 |
| 22 | `worst_other_fill` | max fill among *other* sections (all sections if not PICKUP) | 0..1 |
| 23 | `worst_other_time_to_full` | min ttf among others, /600 | 0..1 |
| 24 | `any_other_blocked` | any other section BLOCKED | 0/1 |
| 25 | `charge_target` | CHARGE: target/100; else 0 | 0..1 |
| 26 | `charge_gain` | CHARGE: max(0, target − battery)/100; else 0 | 0..1 |

**Design notes:**
- The features are **engineered from the symbolic models** (trip energy, battery after going home). The network does not have to re-learn physics.
- **"Others" features** (22–24) give the opportunity cost: if I go to A, how urgent are B and C?
- The docstring of `neural.py` says "25 numbers". **The code has 27.** The docstring is stale.
- Normalisation constants (25 m, 20 %, 600 s, 15 units, 20 units) are hand-chosen. The network also standardises inputs (§5.2).
- **Missing information:** the time the action takes (loading/charging duration), the section's absolute rate, `fault_remaining_s`, which sections the cargo came from, and the charge time for CHARGE. For CHARGE the network must infer duration from `charge_gain` alone.

### 4.3 Real example: the first decision in `low_battery_start` (seed 0)

State: at the charger, battery 30 %. A RUNNING 4/8 (ttf 701 s, 9.04 m), B RUNNING 5/10 (ttf 601 s, 15.18 m), C RUNNING 1/3 (ttf 1463 s, 11.27 m).

| Feature | PICKUP:A | PICKUP:B | CHARGE:70 | WAIT:60 |
|---|---|---|---|---|
| is_pickup/deliver/charge/wait | 1,0,0,0 | 1,0,0,0 | 0,0,1,0 | 0,0,0,1 |
| battery | 0.30 | 0.30 | 0.30 | 0.30 |
| distance | 0.362 | 0.607 | 0.0 | 0.0 |
| energy_cost_percent | 0.019 | 0.032 | 0.0 | 0.0 |
| battery_after | 0.296 | 0.294 | 0.30 | 0.30 |
| battery_after_home | 0.292 | 0.287 | 0.30 | 0.30 |
| target_fill | 0.50 | 0.50 | 0 | 0 |
| target_units_gain | 0.267 (4/15) | 0.333 (5/15) | 0 | 0 |
| target_mass_gain | 0.40 (2.0 kg) | 0.40 (2.0 kg) | 0 | 0 |
| target_time_to_full | 1.0 (701 s > 600) | 1.0 | 1.0 | 1.0 |
| worst_other_fill | 0.50 | 0.50 | 0.50 | 0.50 |
| charge_target / charge_gain | 0 / 0 | 0 / 0 | 0.7 / 0.4 | 0 / 0 |

(Charger distance is 0 for CHARGE because the robot is already there.)

---

## 5. The neural scorer (`neural.py`)

### 5.1 Architecture

```python
class ActionScorer(nn.Module):
    def __init__(self, hidden_sizes=(64, 64), n_features=27):
        layers = [Linear(27, 64), ReLU(), Linear(64, 64), ReLU(), Linear(64, 1)]
        self.net = nn.Sequential(*layers)
        self.register_buffer("mean", torch.zeros(27))
        self.register_buffer("std", torch.ones(27))

    def forward(self, x):
        return self.net((x - self.mean) / self.std).squeeze(-1)
```

**Concept: multilayer perceptron (MLP).** Each `Linear` computes $h = Wx + b$. **ReLU** ($\max(0, z)$) adds non-linearity, so the network can represent interactions such as "far trips are fine when the battery is high but bad when it is low".

**Parameter count:**
- Layer 1: 27 × 64 + 64 = 1 792
- Layer 2: 64 × 64 + 64 = 4 160
- Output: 64 × 1 + 1 = 65
- **Total: 6 017 trainable parameters** (confirmed by loading `ns_scorer.pt`). For comparison, the PPO actor-critic has **14 153** (two separate 40→64→64 tanh towers, an 8-way action head and a value head; confirmed by loading `ppo_policy.zip`).

### 5.2 Input standardisation stored with the weights

```python
def set_normalisation(self, x):
    self.mean.copy_(x.mean(dim=0))
    self.std.copy_(x.std(dim=0).clamp_min(1e-6))
```

**Concept: z-score standardisation.** $\tilde{x} = (x - \mu)/\sigma$ per feature, computed on the training data. Each feature then has mean 0 and std 1, which makes gradient descent well-conditioned. `register_buffer` saves `mean` and `std` in the `state_dict` without making them trainable, so inference always uses the same scaling as training. `clamp_min(1e-6)` avoids division by zero for constant features (e.g. `condition` is always 1.0 in most scenarios).

**Side effect to be aware of:** a feature that was rare or nearly constant in training gets a tiny `std`, so an unusual value produces a **huge standardised input** the network never saw. Printing the stored statistics of the committed model:

| Feature | Training mean | Training std | Standardised value when the feature is 1 |
|---|---|---|---|
| `target_blocked` | 0.0038 | 0.061 | **(1 − 0.0038)/0.061 ≈ 16** |
| `any_other_blocked` | 0.0099 | 0.099 | **≈ 10** |
| `target_fault` | 0.0057 | 0.076 | ≈ 13 |
| `energy_cost_percent` | 0.0196 | 0.015 | a heavy trip at 0.06 → ≈ 2.7 |
| `condition` | 0.904 | 0.208 | worn robot 0.45 → −2.2 (fine: `worn_robot` was in training) |

Blocked lines were **rare** in the training data (0.4–1 % of samples), because the training scenarios were mostly served in time. In scenarios where lines block often (`heavy_load`, `small_buffers`), `any_other_blocked = 1` puts an input at about **10 standard deviations**, far outside the training distribution. That is a plausible mechanism for the model's odd `heavy_load` behaviour (§10), and it can be tested directly.

### 5.3 Save and load with a feature contract

```python
torch.save({"state_dict": ..., "hidden_sizes": [64, 64], "features": FEATURE_NAMES, "extra": metrics}, path)
...
if blob.get("features") != FEATURE_NAMES:
    raise ValueError(f"{path} was trained on different features - retrain it")
```

If someone adds or reorders a feature, the old model is **refused** instead of silently producing garbage. `extra` carries the training metrics. For the committed model: round 1, 13 536 samples, validation top-1 agreement **35.0 %**, train top-1 41.5 %, validation score 59.68, trained on the original 6 scenarios.

### 5.4 Inference

`score(feature_rows)` runs under `@torch.no_grad()` in eval mode on a batch of candidates. Typical decision latency in the fast simulator is about **1.2 ms** (measured mean over a balanced shift). The live service answers in 3–7 ms warm (HANDOVER WP7).

---

## 6. Residual scoring: `symbolic_estimate` + network correction

### 6.1 The symbolic estimate

```python
def symbolic_estimate(state, action, p, matrix, objective):
    d = matrix[location][dest] if dest else 0
    seconds = d / speed
    PICKUP : seconds += load_time;   units = units_that_fit(target)
    DELIVER: seconds += unload_time; units = cargo_units
    WAIT   : seconds += value;       units = 0
    CHARGE :                          units = 0
    energy = trip_energy_wh(d, payload, T, condition)
    if not CHARGE: energy += idle_power × seconds / 3600
    return units × value_per_unit (1.0) − energy × cost_per_wh (0.5)
```

It is "the obvious part of the value": units moved minus energy spent. A PICKUP of 5 units gets about +4.9, a DELIVER of 3 units about +2.9, a WAIT at the charger −0.05, a CHARGE 0.

It deliberately **counts units at PICKUP time as well as at DELIVER time**. That is a heuristic value signal, not the true objective (which pays only on delivery).

### 6.2 The final score

```python
def _neural_scores(self, state, actions, p, matrix):
    rows = [action_features(state, a, p, matrix) for a in actions]
    corrections = self.scorer.score(rows)
    return {a: symbolic_estimate(state, a, ...) + c for a, c in zip(actions, corrections)}
```

$$\text{score}(s, a) = \underbrace{\hat{V}_{sym}(s, a)}_{\text{analytic}} + \underbrace{f_\theta(\phi(s, a))}_{\text{learned correction}}$$

**Concept: residual learning.** The network learns only what the formula misses: which line blocks next, whether a fuller load would have been better, whether the battery will run out later. HANDOVER WP5 recorded that learning the full value lost to the plain rules, while learning the residual raised oracle agreement from about 40 % to 60 % on that early metric.

### 6.3 Reality check with real numbers

For the `low_battery_start` decision in §4.3:

| Action | Symbolic estimate | Network correction | Final score |
|---|---|---|---|
| PICKUP:A | 3.914 | +12.043 | 15.957 |
| **PICKUP:B** | 4.872 | +11.683 | **16.555** ← chosen |
| PICKUP:C | 0.899 | +14.269 | 15.168 |
| CHARGE:40 | 0.000 | +13.343 | 13.343 |
| CHARGE:70 | 0.000 | −12.425 | −12.425 |
| CHARGE:100 | 0.000 | −31.542 | −31.542 |
| WAIT:60 | −0.050 | +14.075 | 14.025 |

Explanation produced: `[routine] B has 5 units waiting; neural score +16.56, chosen over PICKUP:A by +0.60`

**What this shows:**
1. The "correction" is **not small**. It is +12 to +14 for most actions and −31 for CHARGE:100. The network output is on the scale of the look-ahead value (discounted units over 900 s, file 11), while the estimate is on the scale of one action. So in practice the network carries most of the value and the estimate mainly separates *similar* actions (e.g. A vs B by units).
2. The network learned that **charging to 70 % or 100 % at shift start is very bad** (a long non-productive time while buffers are half full). That is exactly the rule policy's failure mode, avoided.
3. The **margin between the top candidates is small** (0.60 here; 0.02–0.35 in many balanced decisions). Given validation top-1 agreement of only 35 %, many decisions are close to a coin flip between similar options. The rules make sure the flip is always between safe, priority-correct options.

---

## 7. The decision procedure (`NeuroSymbolicPolicy.decide`)

```python
def decide(self, state, legal_actions, sim=None):
    p, matrix = sim.p, sim.matrix
    verdicts = self.rules.evaluate(state, legal_actions, p, matrix)

    if mode == "neural":                                   # ABLATION: no rules at all
        scores = neural_scores(all legal actions); return argmax, "neural score +X (no rules)"

    candidates, tier = allowed_by_tier(verdicts)
    if not candidates:                                     # everything forbidden
        action = first legal WAIT/CHARGE (else first legal)
        return action, "every other action is forbidden (<all refusals>)"
    if len(candidates) == 1:                               # rules decided alone
        return that action, "[tier label] reason | refusals"
    if mode == "symbolic":                                 # ABLATION: no network
        scores = {a: -energy_per_unit(a)}; return argmax, "...; cheapest at X Wh per unit"
    scores = neural_scores(candidates)                     # FULL MODEL
    ranked = sorted(scores, reverse=True)
    return ranked[0], "[tier] reason; neural score +S, chosen over <2nd> by +M"
```

### 7.1 Explanations

`_why()` builds `"[<tier label>] <reasons of the chosen action>"` and appends up to **two** refusals of other actions (`" | PICKUP:A refused: battery would be 16 % ..."`). With the score and margin added, one decision explains:
- **which priority level** applied,
- **why this action** (rule reason),
- **why not others** (hard-rule refusals),
- **how confident** (margin over the runner-up).

That is a form of **contrastive explanation** ("why P rather than Q").

Scores that are ±∞ (symbolic mode's "moves no units") are reported as `None`, because JSON has no infinity.

### 7.2 Real example: tier 2 decides alone (no network needed)

`one_hot_section`, seed 2, t = 700 s, robot at delivery. A RUNNING 5/8, **B BLOCKED 10/10**, C 0/3.

```
PICKUP:A    allowed tier 0 | A has 5 units waiting
PICKUP:B    allowed tier 2 | B is BLOCKED and losing production
WAIT:60     allowed tier 0 | waiting away from the charger only spends energy
ns -> PICKUP:B | [unblock a stopped line] B is BLOCKED and losing production | scores {}
```

Only one tier-2 candidate exists, so the network is not consulted (empty `scores`). Symbolic-only gives the same answer.

### 7.3 Real example: the dead band (§3.4) and a questionable choice

This is a constructed state from balanced, seed 3: t = 1500 s, the robot at **B** with 3 units (1.2 kg), **battery 17.8 %**. A BLOCKED 8/8, B BLOCKED 10/10, C RUNNING 1/3.

```
PICKUP:A    FORBIDDEN | battery would be 16 % back at the charger, below the 17 % reserve
PICKUP:B    FORBIDDEN | battery would be 17 % back at the charger, below the 17 % reserve
PICKUP:C    FORBIDDEN | battery would be 17 % back at the charger, below the 17 % reserve
DELIVER     FORBIDDEN | battery would be 17 % back at the charger, below the 17 % reserve
CHARGE:40   allowed tier 0 | top up from 18 %
CHARGE:70   allowed tier 0 | top up from 18 %
CHARGE:100  allowed tier 0 | top up from 18 %
WAIT:60     allowed tier 0 | waiting away from the charger only spends energy
```

Decisions:

| Mode | Action | Explanation (shortened) |
|---|---|---|
| `ns` | **WAIT:60** | `[routine] waiting away from the charger only spends energy \| PICKUP:A refused: ...; neural score +10.78, chosen over CHARGE:40 by +4.55` |
| `ns_symbolic_only` | CHARGE:40 | `[routine] top up from 18 % \| ...; no cheaper alternative` |
| `ns_neural_only` | **PICKUP:B** | `neural score +17.51 (no rules)` (would break the reserve) |

What happened:
- H3 forbids all work (the rules are doing their job).
- CHARGE is **not** tier 2, because 17.8 > 15 + 0.3 (idle-only home %) + 2 = 17.3.
- All remaining candidates are tier 0, so the network chooses, and it prefers **WAIT:60 at B**. The rule's own explanation text says that option "only spends energy".
- The robot will drain 0.45 % per minute until the CHARGE tier-2 threshold forces it home. The robot stays **safe** (CHARGE is always allowed and the trip home costs about 0.64 %), but it wastes minutes while two lines are blocked.
- `ns_symbolic_only` picks `CHARGE:40` only because all its scores are −∞ and `max()` returns the first key in legal-action order (CHARGE:40 comes before WAIT). That is an accident of ordering, not a reasoned choice.

**Lesson:** the rules guarantee safety, but they do not guarantee sensible behaviour inside a tier. Rule gaps show up as network mistakes.

### 7.4 Real example: the "milk run" in balanced (seed 0)

```
t=   0.0 PICKUP:B  [routine] B has 2 units waiting; neural score +8.67, chosen over PICKUP:A by +0.04
t=  68.9 PICKUP:A  [routine] A has 1 units waiting; neural score +9.73, chosen over DELIVER by +0.35
t= 145.8 DELIVER   [routine] carrying 3 units; neural score +11.78, chosen over PICKUP:B by +0.30
t= 201.5 PICKUP:B  [routine] B has 1 units waiting; neural score +8.36, chosen over PICKUP:A by +0.17
t= 257.8 DELIVER   [routine] carrying 1 units; neural score +8.44, chosen over PICKUP:A by +0.05
t= 314.1 PICKUP:B  [routine] B has 1 units waiting; ...
t= 370.4 DELIVER   [routine] carrying 1 units; ...
```

The model shuttles single units from B (10.3 m from delivery) every ~113 s. Shift result: **50 delivered, 0 lost, 9.0 Wh, 527 m, 0 charge trips, score 45.5**. The rule policy: 43 delivered, 0 lost, 4.1 Wh, 290 m, score 40.9.

Under the objective this is better (file 08 §4.2: the end-of-shift effect plus cheap energy). As "resource-efficient navigation" it is questionable: **2.2× the energy and 1.8× the distance for the same zero loss.**

---

## 8. The three modes (ablations)

| Mode | Name | Hard rules | Priority tiers | Ranking | Question it answers |
|---|---|---|---|---|---|
| `full` | `ns` | ✔ | ✔ | symbolic estimate + network | the proposed model |
| `neural` | `ns_neural_only` | ✘ | ✘ | estimate + network over **all legal** actions | what does the network alone do? how much safety do the rules add? |
| `symbolic` | `ns_symbolic_only` | ✔ | ✔ | lowest **Wh per unit moved** (−∞ for CHARGE/WAIT) | what do the rules alone achieve? what does the network add? |

**Missing-model behaviour:** if `ns_scorer.pt` is missing or was trained on different features, `full`/`neural` switch to `symbolic` and add ` | no trained scorer (...); running on rules alone` to every explanation.

**Note on the neural ablation:** it still uses the **same network trained on data labelled only for rule-allowed candidates** (file 11). Unsafe actions were never labelled, so the network has never seen what happens after them. "Neural only" therefore measures *this network outside its training distribution*, not the best possible rule-free network.

---

## 9. Tests that pin the architecture

`test_symbolic.py`:
- refuses a trip that would strand the robot; allows the same trip with a full battery
- refuses work when maintenance is needed; refuses a pickup that does not fit; refuses a route that would overheat
- the hard rules use the robot's own models (e.g. a heavier payload changes the verdict)
- a blocked line is top priority; a line about to block outranks a quiet one; an empty battery makes charging top priority
- every verdict explains itself; `allowed_by_tier` keeps only the most urgent

`test_neurosymbolic.py`, the important ones:
- **`AlwaysPicksTheWorst`**: a deliberately sabotaged scorer, `score = distance × 10 − battery_after_home × 100`, which prefers long trips that end with the least battery.
  - `test_a_bad_network_cannot_break_the_battery_rule`
  - `test_a_bad_network_cannot_make_the_robot_work_when_it_needs_maintenance`
  - `test_a_whole_shift_with_a_bad_network_stays_safe`: zero violations, the battery never drops below reserve
- `test_the_network_only_chooses_inside_the_top_priority_tier`
- `test_neural_only_ignores_the_rules`, `test_symbolic_only_needs_no_network`, `test_missing_model_falls_back_to_rules_and_says_so`
- `test_every_decision_of_a_whole_shift_is_explained` for **all 12 scenarios**
- `test_decisions_are_fast_enough_for_the_robot` (< 100 ms)
- `test_scorer_saves_and_loads_with_its_normalisation`, `test_scorer_refuses_a_model_trained_on_other_features`

The sabotage tests are the strongest evidence for the thesis claim: **the safety property is structural, not learned.**

---

## 10. Critical assessment

**Strong points**
- A clear separation of *must* (hard rules), *should first* (tiers) and *judgement* (network).
- Safety that does not depend on training quality, with a test that proves it.
- Every decision explained in words, with contrastive refusals and a confidence margin.
- The same code offline and live, fast (about 1 ms), small (6 k parameters).
- A missing model degrades to rules automatically.

**Weak points (verified in the code and in runs)**
1. **Rule coverage gaps:** WAIT away from the charger is unconstrained; the CHARGE tier uses idle-only energy (a dead band against H3); DELIVER tier 1 only when *exactly* full.
2. **Weak ranking accuracy:** validation top-1 agreement with the look-ahead oracle is 35 % (round 1, kept), and later rounds were no better (file 11). Many decisions are decided by margins below 0.5.
3. **Out-of-distribution scenarios:** trained on 6 scenarios. In the 6 new ones it does well in some (`aged_battery`) and badly in `heavy_load`, where the network repeatedly chose `WAIT:60` with cargo on board over `DELIVER` (14 WAITs in one shift), losing 26 units (file 13).
4. **Energy use:** about 1.5–2× the rule policy's energy in most scenarios. The objective does not punish it enough.
5. **Priority tiers are hand-set and absolute.** A BLOCKED section 18 m away always beats a line that will block in 30 s next door, even if the second is the better choice.
6. **One-step safety.** H3 guarantees a way home after *this* action. It does not plan a sequence to minimise lost production while staying safe.
7. **Standardised inputs far out of distribution** for rare events: a blocked line gives inputs of 10–16 standard deviations (§5.2).
8. **Stale docstring** in `neural.py` (25 vs 27 features).

---

## 11. Key concepts of this section

- Neuro-symbolic integration patterns; symbolic shield + neural ranker
- Hard constraints vs soft preferences; lexicographic priorities
- Forward simulation as a symbolic reasoning tool (model-based constraint checking)
- Action-conditioned value functions (Q-style scoring of candidates)
- Feature engineering from domain models
- MLP, ReLU, z-score standardisation, buffers in PyTorch
- Residual learning over an analytic prior
- Contrastive, rule-grounded explanations
- Ablation studies
- Adversarial (sabotage) testing of a safety architecture

---

## 12. Improvements and technologies for this section

Ordered by expected value for the thesis.

| # | Idea | Why | How |
|---|---|---|---|
| 1 | **Close the rule gaps** | The dead band and unconstrained WAIT cause visible bad decisions | (a) CHARGE tier 2 must use the same `simulate_route` home energy + margin as H3, so "all work forbidden ⇒ CHARGE is tier 2"; (b) forbid WAIT away from the charger when cargo > 0 and DELIVER is allowed, or add it as a tier −1 (last resort) |
| 2 | **Soft tiers instead of hard tiers** | Absolute tiers ignore distance and timing trade-offs between a blocked line far away and one about to block nearby | Keep hard rules as a shield, but feed the tier as a *feature* and let the network rank across tiers, or add a large-but-finite bonus per tier (e.g. +5 per tier) |
| 3 | **Retrain on all 12 scenarios with domain randomisation** | OOD failures (`heavy_load`) | Randomise rates, masses, buffers, battery, ambient per training episode from ranges around the scenarios; evaluate on held-out scenario combinations |
| 4 | **Uncertainty-aware ranking** | Small margins with 35 % oracle agreement | Train an **ensemble** of 5 scorers (or MC-dropout); if the ensemble disagrees, fall back to the symbolic ranking and say so in the explanation ("low confidence") |
| 5 | **Set-based / attention scorer** | Candidates are scored independently; interactions ("A now, then B on the way") are invisible | A small Transformer / Deep Sets encoder over all candidate feature vectors, outputting a score per candidate (listwise scoring); still behind the same shield |
| 6 | **Multi-step symbolic safety** | One-step H3 | Check the best 2-action sequence (e.g. PICKUP then DELIVER then home) with `simulate_route`, or plan short sequences with the rules as constraints |
| 7 | **Formal rule specification** | Rules live in Python `if`s; hard to audit or prove | Express hard rules in a declarative form: **Answer Set Programming** (`clingo`), **Datalog**, or **PDDL** preconditions; prove properties with **SMT** (`z3`), e.g. "no allowed action leaves the battery below reserve at the charger" |
| 8 | **Probabilistic logic** | Rules are crisp; production is stochastic | `ProbLog` / `DeepProbLog` / Scallop: rules with probabilities ("B blocks within travel time with p = 0.7"), with a network providing probabilistic facts, end-to-end differentiable |
| 9 | **Explanation quality evaluation** | Explanations exist but are not evaluated | Small user study (do operators predict the robot's next action better with explanations?) or faithfulness tests (does removing the cited reason change the decision?) |
| 10 | **Feature additions** | Missing action duration, absolute rates, fault time left, charge time | Add `action_duration_s/600`, `charge_time_s/3600`, `rate_nominal/60`, `fault_remaining_s/1800`; retrain (the feature contract forces it) |
| 11 | **Robust input scaling** | Rare binary features become 10–16σ inputs | Do not standardise binary/one-hot features (they are already 0/1), clamp standardised inputs to ±5, and make sure training data contains enough blocked/fault states (oversample hard scenarios) |
