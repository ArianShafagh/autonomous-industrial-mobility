# 08 — The fast simulator, the decision problem, and the Gymnasium environment

Files:
- `src/robofetch_ai/robofetch_ai/env/factory_sim.py` (293 lines): `FactorySim`, `Outcome`, `run_episode`
- `src/robofetch_ai/robofetch_ai/env/factory_env.py` (100 lines): `FactoryEnv` (Gymnasium)
- `tools/ai/validate_against_gazebo.py`: sim vs Gazebo check
- `params.yaml → mission.objective`, `mission.simulation`
- Tests: `test_factory_sim.py` (16), `test_factory_env.py` (8)

This is the **training ground** of the AI. If you want to change the AI, you will spend most of your time here and in files 10–13.

---

## 1. Why a second simulator

| | Gazebo + Nav2 | `FactorySim` |
|---|---|---|
| One 1-hour shift | ~30 min wall time (headless) | **~0.07 s** with the rule policy (neuro-symbolic: ~0.1–0.2 s, because the rules forward-simulate routes) |
| Physics, lidar, localisation, controller | real | replaced by `distance / speed` |
| Robot energy/heat/wear | `robot_model.py` via `robot_state_node` | **same** `robot_model.py` |
| Production | `factory_model.py` via `factory_node` | **same** `factory_model.py` |
| Distances | driven by Nav2 | **same** `path_matrix.yaml` |
| Actions | executor | **same** `mission_plan.Action` |
| Navigation failures | can happen | **not modelled** |

Training a neuro-symbolic model needs about 13 000 labelled decision samples, each requiring several look-ahead shifts. PPO needs 300 000 steps, about 5 000 shifts. **That is impossible in Gazebo and trivial in `FactorySim`.**

This is the standard **surrogate model / digital twin** approach. Its validity rests on one condition: the fast simulator must agree with the real system. That is checked in §7.

---

## 2. The decision problem, formally

### 2.1 Markov Decision Process (MDP)

An MDP is a tuple $(S, A, P, R, \gamma)$:
- $S$: states. Here: time, robot location, battery, temperature, condition, payload, cargo, and per section buffer, status, health, time to full, and so on.
- $A(s)$: actions available in state $s$. Here: the legal subset of PICKUP A/B/C, DELIVER, CHARGE 40/70/100, WAIT 60.
- $P(s' \mid s, a)$: transition probabilities. Random because of production noise and faults.
- $R(s, a, s')$: reward. Here: the objective of §4.
- $\gamma$: discount factor, how much future reward counts.

**Goal:** find a policy $\pi(a \mid s)$ maximising the expected return $\mathbb{E}\left[\sum_t \gamma^t r_t\right]$.

### 2.2 It is really a *semi*-Markov decision process (SMDP)

Actions take **different amounts of time**: a WAIT is 60 s, a PICKUP:B from the charger about 69 s, a CHARGE:100 from 30 % about 50 minutes. Time between decisions is variable. That is an **SMDP**, the natural model for macro-actions (options).

How the code handles it:
- `FactorySim.step(action)` runs the action **to completion** and returns an `Outcome` with its `duration_s`.
- `train_ns.py` discounts **by elapsed time**: $\gamma_{100s}^{\Delta t / 100}$ (file 11), which is the correct SMDP treatment.
- `FactoryEnv` for PPO discounts **per step** ($\gamma = 0.995$ per decision regardless of duration). This ignores the variable duration, so PPO values a 50-minute CHARGE the same as a 60 s WAIT in terms of discounting. That is a simplification (file 12).

### 2.3 Partial observability

The policies see `state()`, which is almost the full simulator state. Hidden are the exact remaining work of each future unit, upcoming fault times, and repair durations. Strictly this is a POMDP, but the observable part carries most of the useful information.

---

## 3. `FactorySim` in detail

### 3.1 Construction and reset

```python
sim = FactorySim(scenario="balanced", seed=None, config_dir=None, overrides=None, shift_duration_s=None)
```

It loads the config (params + scenario + overrides), `RobotParams`, the objective, the path matrix, the seed (default `time.seed` = 1), the shift length (3600 s) and the step `dt = mission.simulation.step_s` = 1.0 s.

`reset(seed)` rebuilds the sections from the seed, creates a fresh `RobotCondition`, and sets location = charger, payload 0, cargo/delivered per section 0, distance 0, charge trips 0, violations [], history [], `done = False`, min battery = initial.

### 3.2 `state()`: what a policy may see

```python
{
  "time_s": 145.8, "time_left_s": 3454.2,
  "location": "A",
  "battery_percent": 98.9, "temperature_c": 25.1, "condition_percent": 100.0,
  "payload_kg": 1.3, "cargo_units": 3, "free_payload_kg": 3.7,
  "distance_to_delivery_m": 10.01, "distance_to_charger_m": 9.04,
  "sections": {
    "A": { ...Section.snapshot()..., "distance_m": 0.0,  "units_that_fit": 0 },
    "B": { ..., "distance_m": 18.28, "units_that_fit": 1 },
    "C": { ..., "distance_m": 15.56, "units_that_fit": 0 }
  }
}
```

The snapshot fields are `status`, `status_code`, `produced_total`, `picked_total`, `buffer_units`, `buffer_capacity`, `buffer_fill`, `unit_mass_kg`, `buffer_mass_kg`, `rate_nominal_per_hour`, `rate_actual_per_hour`, `health_percent`, `time_to_full_s`, `fault_remaining_s`, `faults_total`, `lost_units`, `blocked_time_s`, `factory_time_s`.

These are **exactly the facts the live system publishes** (SectionStatus + telemetry). `LiveState` (file 14) rebuilds the same dict from ROS messages, so a policy cannot tell which world it is in.

```python
def units_that_fit(self, sid):
    unit = section.params.unit_mass_kg
    free = max(0, max_payload_kg - payload_kg)
    return min(section.buffer, floor(free / unit + 1e-9))
```

### 3.3 `_advance(seconds, distance_m, motor_load, docked)`: the time integrator

```python
speed = distance_m / seconds
while left > 0 and not self.done:
    step = min(self.dt, left)
    self.robot.step(step, distance_m=speed * step, payload_kg=self.payload_kg, motor_load=motor_load, docked=docked)
    for s in self.sections.values(): s.step(step)
    self.time_s += step
    min_battery = min(min_battery, battery)
    if battery <= 0:          violation("battery_empty"); done = True
    if temperature >= max_c:  violation("overheated")
    if time_s >= shift_duration_s: done = True
return (energy drawn, energy charged, lost units) during this advance
```

The robot and all sections advance **together** in 1 s steps. That is what makes "while you drive to B, A blocks" happen.

### 3.4 `step(action)`: execute one action to completion

```
if done: return Outcome(ok=False, "shift over")
dest = action.destination(); distance = matrix[location][dest] × nav_overhead_factor
1. if distance > 0: _advance(distance / speed, distance, drive_load=0.7); location = dest
2. PICKUP : _advance(load_time 30 s, dwell_load 0); units = section.pickup(max_mass = 5 - payload)
            payload += units × unit_mass; cargo[target] += units; ok = units > 0
   DELIVER: _advance(unload_time 30 s); units = sum(cargo); delivered += cargo; cargo = 0; payload = 0; ok = units > 0
   CHARGE : charge_trips += 1; while battery < min(100, target) and not done: _advance(dt, docked=True)
   WAIT   : _advance(value, dwell_load, docked = (location == charger))
3. if battery < reserve and location != charger and not done: violation("below_reserve")
4. return Outcome(action, ok, detail, units, duration, distance, energy, charged, lost, new violations)
```

**Things to notice:**
- A PICKUP that finds nothing (the buffer emptied by the time you arrive) is `ok = False` but still costs time and energy. That is realistic.
- `charge_trips` counts every CHARGE action, even when the robot is already at the charger.
- The `below_reserve` check happens **only at the end of an action** and **only away from the charger**. Dipping below reserve during a drive that ends at the charger is not a violation.
- `WAIT` away from the charger drains idle power and is **never** a safety violation by itself, unless the battery ends below reserve.

### 3.5 Violations and their deduplication

```python
def _violation(self, kind):
    if not self.violations or self.violations[-1][0] != kind:
        self.violations.append((kind, round(self.time_s, 1)))
```

A violation is recorded **only if it differs from the last one recorded**. Consequences:
- `overheated` checked every 1 s while hot is recorded **once per hot period**, not per second.
- **But also:** a robot that stays below reserve for 20 consecutive actions gets **one** `below_reserve`, while one that alternates "below reserve" and "overheated" gets many. The penalty (−20 each) therefore does not scale with how long or how badly the constraint was violated. File 13 shows PPO exploiting this in `heavy_load`: one violation (−20) for driving the battery to 0.8 %, while delivering about 30 more units than the rule-bound model.

### 3.6 `legal_actions()`: the action mask

```python
for sid in sections:  if units_that_fit(sid) > 0: legal PICKUP:sid
if cargo > 0:          legal DELIVER
for target in [40, 70, 100]:  if battery < target - 1: legal CHARGE:target
legal WAIT:60          (always)
```

**Concept: action masking.** Some actions are pointless in some states. Masking them:
- speeds up learning (PPO never wastes samples on them),
- avoids "illegal action" handling,
- encodes **domain knowledge** as constraints rather than something to learn.

The *same* mask function (and its live twin in `LiveState`) is used by the rule policy, the symbolic layer, the neural training, PPO (via `action_masks()`) and the live service.

This mask only removes **pointless** actions. It does **not** remove **unsafe** actions (a PICKUP that would strand the robot is still legal). Safety is the symbolic layer's job (file 10). This separation is important for the PPO comparison: PPO gets the pointless-action mask but not the safety rules.

---

## 4. The objective: what "good" means

```yaml
objective:
  value_per_unit_delivered: 1.0
  cost_per_lost_unit: 1.0
  cost_per_wh: 0.5
  cost_per_safety_violation: 20.0
  cost_per_second_idle: 0.0          # NOTE: defined but not used by any code
```

Per action (`reward`, also PPO's reward):

$$r = 1.0 \cdot \text{units}_{\text{DELIVER}} - 1.0 \cdot \text{lost}_{\Delta} - 0.5 \cdot E_{\Delta} - 20 \cdot |\text{violations}_{\Delta}|$$

Per shift (`score`, what policies are ranked by):

$$\text{score} = \text{delivered} - \text{lost} - 0.5 \cdot E_{total} - 20 \cdot |\text{violations}|$$

The sum of per-action rewards equals the score. `test_rewards_follow_the_objective_weights` checks the weights.

### 4.1 Worked example (real numbers)

Balanced, seed 0, 1 h:

| Policy | Delivered | Lost | Energy | Violations | Score |
|---|---|---|---|---|---|
| neuro-symbolic | 50 | 0.0 | 9.001 Wh | 0 | 50 − 0 − 4.50 − 0 = **45.50** |
| rule | 43 | 0.0 | 4.118 Wh | 0 | 43 − 0 − 2.06 − 0 = **40.94** |

### 4.2 What the objective rewards, and a trap to know about

1. **Delivered units dominate.** Energy is cheap: 1 Wh = 0.5 units, and a whole shift uses 4–10 Wh, so 2–5 units.
2. **Units still in a buffer or on the robot at shift end count for nothing.** In the example, no line blocked for either policy (lost = 0 for both), so production was identical, about 48–51 units. The 7-unit difference is mostly **how many units were delivered before the 3600 s cut-off**. The rule policy leaves units waiting in buffers until they are worth a trip. The neuro-symbolic model runs many 1-unit trips (a "milk run", see its trace in file 10) and uses **2.2× the energy** to empty buffers early. Under this objective that wins. Under a "no lost production at minimum energy" objective it would lose. **A thesis claim about "resource efficiency" must take this end-of-shift effect into account** (file 13 suggests fixes).
3. **Safety is a soft penalty.** −20 per (deduplicated) violation means a policy without hard rules can find it profitable to violate (PPO in `heavy_load`).

---

## 5. `run_episode`: the shift loop used everywhere

```python
def run_episode(policy, scenario="balanced", seed=0, config_dir=None, overrides=None,
                shift_duration_s=None, trace=False):
    sim = FactorySim(scenario, seed, config_dir, overrides, shift_duration_s)
    while not sim.done:
        state, legal = sim.state(), sim.legal_actions()
        decision = policy.decide(state, legal, sim)          # the policy may use sim for look-ahead
        outcome = sim.step(decision.action)
        if trace: decisions.append({...time, action, why, units, battery, latency_ms, scores})
    return sim.summary(), decisions
```

`summary()` returns: scenario, seed, shift_s, delivered units (total and per section), produced, lost (total and per section), blocked time per section, energy, Wh per unit, distance, charge trips, min/end battery, condition, violations (`kind@time`), number of actions, score.

**Caution:** `sim` is passed to `decide()`. A policy *could* cheat by cloning it and simulating the true future. None of the evaluated policies do that at decision time. The neuro-symbolic policy only reads `sim.p` and `sim.matrix`, and PPO reads the observation. The training script does use clones, which is legitimate there.

---

## 6. `FactoryEnv`: the Gymnasium wrapper (for PPO)

### 6.1 Concept: the Gymnasium API

```python
obs, info = env.reset(seed=None)
obs, reward, terminated, truncated, info = env.step(action_index)
env.action_space        # Discrete(8)
env.observation_space   # Box(low=-1, high=2, shape=(40,), float32)
```

`terminated` means the episode ended naturally (here: shift over or battery empty). `truncated` means it was cut by a time limit (always False here, because the shift end is part of the task).

### 6.2 Actions (fixed order)

`[PICKUP:A, PICKUP:B, PICKUP:C, DELIVER, CHARGE:40, CHARGE:70, CHARGE:100, WAIT:60]`, so **8 discrete actions**. (HANDOVER WP4 says "6 actions". That was before charging became three levels.)

### 6.3 Observation: 40 numbers, roughly 0..1

| Index | Feature | Scaling |
|---|---|---|
| 0 | battery | /100 |
| 1 | temperature | (T − ambient) / (max − ambient) |
| 2 | condition | /100 |
| 3 | payload | / max_payload |
| 4 | time left | / shift length |
| 5–9 | location one-hot | charger, delivery, A, B, C |
| then per section A, B, C (10 each): | | |
| +0 | buffer fill | 0..1 |
| +1 | actual/nominal rate | 0..1 |
| +2 | time to full | min(1, s/600), inf → 1 |
| +3 | health | /100 |
| +4 | distance from current location | / longest matrix entry (18.28 m) |
| +5 | units that fit | / buffer capacity |
| +6..+9 | status one-hot | RUNNING, DEGRADED, BLOCKED, FAULT |

Total: 5 + 5 + 3 × 10 = **40**. (HANDOVER says "~28-value observation". The code gives 40, confirmed by loading the trained PPO model: `Linear(in_features=40, ...)`.)

**Why scale to ~0..1?** Neural networks train poorly when inputs differ by orders of magnitude (battery 0–100 vs fill 0–1). SB3 does not normalise by default, so manual scaling matters.

**What PPO does not see:** the absolute rates (only the ratio actual/nominal), `unit_mass_kg`, `fault_remaining_s`, `lost_units`, cargo count, and which section the cargo came from. The scenario differences are visible only indirectly (a scenario with double rates looks like "fills faster"). The observation is **not** fully Markov across scenarios.

### 6.4 `step()`

```python
action = self.actions[action_index]
legal = {str(a) for a in sim.legal_actions()}
outcome = sim.step(action)
reward = sim.reward(outcome)
if str(action) not in legal:
    reward -= 0.1          # illegal choices execute anyway (achieve nothing) but cost a little
terminated = sim.done
```

The −0.1 lets an **unmasked** agent (ablation) learn the rules instead of exploiting them. An illegal PICKUP still drives to the section and loads 0 units, wasting time.

### 6.5 `reset()` and seeds

With `randomise_seed=True` (default) and no explicit seed, every reset draws a new seed from the env's own RNG. PPO therefore trains on a new random factory every episode, which is good for generalisation. The scenario is fixed per env instance (see the scenario-coverage bug in file 12).

### 6.6 `action_masks()`

Returns a boolean array over the 8 actions. `sb3_contrib.MaskablePPO` reads it through the `ActionMasker` wrapper and sets masked logits to −∞ before the softmax.

---

## 7. Validation against Gazebo (`validate_against_gazebo.py`)

**Method:** take a finished Gazebo mission CSV, keep the `SUCCEEDED` rows, rebuild each action (CHARGE uses the *predicted end battery* as its target, WAIT uses the *measured duration*), replay them in a `FactorySim` with the same scenario and seed and an effectively infinite shift, and compare distance, duration and energy per action and in total. It fails if the worst total difference exceeds **15 %**.

**Result (HANDOVER WP4, run `run_20260915_125858`, 7 actions):**

| Total | Gazebo | Fast sim | Diff |
|---|---|---|---|
| distance | 61.17 m | 61.17 m | +0.0 % |
| duration | 526.4 s | 529.1 s | +0.5 % |
| energy | 0.844 Wh | 0.849 Wh | +0.6 % |

The worst single action was PICKUP:B, 62.6 s in Gazebo vs 68.9 s in the sim (+10 %).

**Limits of this validation (important for a thesis):**
- It uses **one short mission** with no failures. It validates the *cost model*, not the *production outcome*: units picked up can differ because production is random and the replay does not reuse Gazebo's section RNG state.
- Distances match exactly partly **by construction**: the fast sim uses the matrix, and Gazebo's driven distance happened to match the matrix at 0.0 %.
- Energy agreement follows from both sides using `robot_model.py`. What is really being validated is that Nav2 drives about the matrix distance at about 0.39 m/s.
- A stronger validation would compare **whole-shift outcomes** (delivered, lost, energy) of the *same policy* in Gazebo and in the sim over several seeds.

---

## 8. Tests that pin this behaviour

`test_factory_sim.py`:
- units conserved (produced + initial = buffer + cargo + delivered)
- energy equals the robot model's energy; distances equal the path matrix
- payload limit respected; deliver empties the robot
- charge reaches target and takes time; waiting at the charger charges, elsewhere drains
- legal actions refuse pointless ones; empty section cannot be picked up
- running out of battery is a violation and ends the shift; working below reserve is recorded
- same seed → same shift, other seed differs; the shift ends at the configured length; an episode is fast

`test_factory_env.py`: passes `gymnasium.utils.env_checker.check_env`, observation shape and range, charging levels are separate actions, mask equals simulator legal actions, episode ends with a summary, illegal actions are penalised but do not crash, same seed gives the same trajectory, rewards follow the objective weights.

---

## 9. Key concepts of this section

- Surrogate simulation / digital twin; sim-to-real gap; validation
- MDP, SMDP (variable-duration actions), POMDP
- Macro-actions / options
- Reward function vs evaluation metric (here identical by design)
- Action masking vs safety constraints (different jobs)
- Episode termination vs truncation (Gymnasium API)
- Observation normalisation
- Terminal (end-of-horizon) effects in finite-horizon objectives
- Constraint handling by penalty (soft) vs by rule (hard)

---

## 10. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Terminal value for leftover work** | Units in buffers/cargo at shift end count as 0, which rewards last-minute emptying and biases comparisons | Add `+ value × (cargo + buffers) × α` at `done`, or evaluate on a rolling horizon (e.g. 8 h shift, score the middle 6 h), or count "lost + backlog" |
| **Violation penalty proportional to severity/duration** | Deduplication makes a long stretch below reserve cost the same as a momentary dip | Penalise seconds below reserve (× how far below), and every overheated second; or treat safety as a **constraint** (CMDP) instead of a penalty |
| **Time-based discount in `FactoryEnv`** | PPO discounts per decision, not per second | Return `info["duration_s"]` and use an SMDP-aware return: `γ^(Δt/τ)`; SB3 does not support per-step γ, so either a custom rollout buffer or reward shaping with duration |
| **Richer PPO observation** | Scenario-specific quantities (absolute rates, unit mass, fault time left, cargo) are hidden | Add them (normalised), or give PPO the same 27 per-action features in a "set of candidates" architecture for a fairer comparison |
| **Vectorise / speed up** | Neuro-symbolic training takes ~40 min, mostly deep-copying the simulator for rollouts | Profile (`py-spy`), avoid `copy.deepcopy` (explicit lightweight clone), `numba` for section stepping, or run rollouts in parallel with `multiprocessing` / `ray` |
| **Stochastic navigation** | Constant speed and zero failures make the sim optimistic | Sample drive time from the Gazebo log distribution (e.g. lognormal around d/v), add a blockage probability and a recovery-time cost |
| **Stronger sim-vs-Gazebo validation** | One 7-action replay | Run the same policy for full shifts on 5 seeds in both; compare delivered/lost/energy distributions (two-sample test), and report the gap as a thesis result |
| **Use `cost_per_second_idle` or remove it** | Defined in `params.yaml`, read by no code; misleading | Delete it or implement it in `reward()` / `score()` |
| **PettingZoo / multi-agent version** | Future multi-robot work | Wrap `FactorySim` with several robots as a PettingZoo `ParallelEnv` |
