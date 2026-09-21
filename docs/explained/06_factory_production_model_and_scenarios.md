# 06 — Factory production model and scenarios

Files:
- `src/robofetch_factory/robofetch_factory/factory_model.py` (281 lines, pure Python): `Section`, `SectionParams`, `load_config`, `build_sections`
- `src/robofetch_factory/robofetch_factory/factory_node.py`: live ROS node for all three sections
- `src/robofetch_factory/robofetch_factory/monitor.py`: terminal view
- `src/robofetch_factory/config/params.yaml → factory:` and `config/scenarios/*.yaml`
- `scripts/param_report.py`: capacity and utilisation analysis
- Tests: `src/robofetch_factory/test/test_factory_model.py` (34+ tests incl. parametrised)

This model **creates the demand** the robot must serve. Its randomness is why the decision problem is hard, and why a policy must be evaluated over many seeds.

---

## 1. A section = machine + output buffer

```
  work accumulates        a unit completes       the robot empties
  at rate × health  ──►   when work ≥ needed ──► the buffer (PICKUP)
        ▲                        │
        │                        ▼
   FAULT stops it          BUFFER [■■■■□□□□]  capacity 8
                                 │ full?
                                 ▼
                     BLOCKED: machine stops, production counted as LOST
```

Four state flags produce one **status**, checked in this priority order (`Section.status`):

```python
if self.repair_remaining_s > 0.0:                    return FAULT      # broken, under repair
if self.buffer >= self.params.buffer_capacity:       return BLOCKED    # full, stopped, losing
if self.health < self.params.degraded_below:         return DEGRADED   # worn, slower
return RUNNING
```

Status codes (in `SectionStatus.msg`): RUNNING 0, DEGRADED 1, BLOCKED 2, FAULT 3.

---

## 2. Parameters

`SectionParams` has no defaults. Values come from `factory.sections.<id>` merged over `factory.section_defaults`:

| Key | A | B | C | Meaning |
|---|---|---|---|---|
| `rate_per_hour` | 20 | 30 | 5 | nominal units per **factory** hour (user requirement) |
| `buffer_capacity` | 8 | 10 | 3 | units; = 24 / 20 / 36 min of output |
| `unit_mass_kg` | 0.5 | 0.4 | 1.5 | full buffer = 4.0 / 4.0 / 4.5 kg, all ≤ 5 kg payload |
| `initial_buffer` | 1 | 2 | 0 | at shift start |
| `production_cv` | 0.15 | 0.15 | 0.15 | coefficient of variation of cycle work |
| `health_initial` | 100 | 100 | 100 | % |
| `wear_per_unit` | 0.1 | 0.1 | 0.1 | health % lost per unit produced |
| `degraded_below` | 60 | 60 | 60 | % |
| `min_rate_factor` | 0.5 | 0.5 | 0.5 | slowest speed when health → 0 |
| `faults_per_hour` | 0.1 | 0.1 | 0.1 | healthy machine: MTBF 10 h |
| `fault_wear_gain` | 3.0 | 3.0 | 3.0 | hazard multiplier for wear |
| `repair_minutes` | 20 | 20 | 20 | mean time to repair |
| `repair_cv` | 0.3 | 0.3 | 0.3 | variation of repair time |

**Design rule verified by a test:** `test_a_full_buffer_always_fits_on_the_robot` checks that in every scenario one trip can always unblock a line, i.e. capacity × unit mass ≤ 5 kg.

---

## 3. Production with Gamma-distributed work

### 3.1 Concept: the Gamma distribution parameterised by mean and CV

Real machine cycle times vary. They are positive, usually close to a mean, and right-skewed. The **Gamma distribution** fits that. With shape $k$ and scale $\theta$, the mean is $k\theta$ and the variance is $k\theta^2$, so the coefficient of variation (CV = std/mean) is $1/\sqrt{k}$. To get a chosen mean $\mu$ and CV:

$$k = \frac{1}{CV^2}, \qquad \theta = \frac{\mu}{k}$$

```python
def _gamma(rng, mean, cv):
    if cv <= 0.0:
        return mean                            # deterministic
    shape = 1.0 / (cv * cv)
    return rng.gammavariate(shape, mean / shape)
```

With CV 0.15 the shape is 44.4, which is nearly normal: 95 % of cycles fall within about ±30 % of the mean. Five real samples (seed 0): 1.198, 0.966, 1.005, 1.147, 0.990.

CV = 0.15 describes a typical automated line. CV = 1 would give an exponential distribution, which is very erratic.

### 3.2 Work units instead of time

Each unit needs `_work_needed ~ Gamma(mean 1, cv)` "units of work". The machine accumulates work at `speed = rate_per_hour / 3600 × rate_factor` per factory second. When accumulated `_progress` reaches `_work_needed`, a unit is done.

### 3.3 Event-by-event integration (step-size independence)

```python
while dt > 0.0:
    if self.buffer >= p.buffer_capacity:               # blocked for the rest of the step
        self.blocked_time_s += dt
        self.lost_units += rate_per_s * self.rate_factor() * dt
        return
    speed = rate_per_s * self.rate_factor()
    t_unit = (self._work_needed - self._progress) / speed     # time until this unit completes
    if t_unit > dt:                                     # not in this step
        self._progress += speed * dt
        return
    dt -= t_unit                                        # finish the unit exactly at its time
    self.buffer += 1; self.produced_total += 1
    self.health -= p.wear_per_unit                      # wear applies at that instant
    self._progress = 0.0
    self._work_needed = _gamma(self.rng, 1.0, p.production_cv)
```

**Why loop inside a step?** A naive `if progress >= needed: buffer += 1` would allow at most one unit per step and put the blocking moment at the step boundary. The results would then depend on the step size: the live node steps every 0.1 s, the fast simulator every 1 s, and a test uses 2 s. Looping on events makes the dynamics **exact regardless of step size**. That is verified by `test_result_does_not_depend_on_step_size` (step 2.0 s vs 0.1 s give identical results).

This is a small **discrete-event simulation** embedded inside a time-stepped one.

---

## 4. Blocking and lost production

When the buffer is full, the machine is **blocked** (in manufacturing terms, "blocking after service"). The model counts what it *would* have produced:

$$\text{lost} \mathrel{+}= \frac{\text{rate}}{3600} \cdot \text{rate\_factor} \cdot \Delta t_{blocked}$$

Lost units are **fractional** (e.g. 0.0178 after 2.1 s blocked). They are the key cost in the objective: −1 per lost unit, the same weight as +1 per delivered unit.

**Real example (seed 1, balanced, section B, nobody collecting):**

| | t = 0 | t = 900 s |
|---|---|---|
| status | RUNNING | **BLOCKED** |
| buffer | 2 / 10 | 10 / 10 |
| produced | 0 | 8 |
| health | 100 % | 99.2 % |
| time_to_full_s | 938.4 | 0 |
| lost_units | 0 | 0.018 (blocked 2.1 s) |

Verified live in Gazebo (HANDOVER WP2, old ts 20): B blocked 96.35 s at 60 units/h × time scale 20 gave **32.1 lost units** = 96.35 × 20 / 3600 × 60, exact.

---

## 5. Machine health and the degraded rate

Every produced unit costs `wear_per_unit` = 0.1 % health. Below `degraded_below` (60 %) the machine slows:

$$\text{rate\_factor} = \begin{cases} 1 & h \ge 60 \\ f_{min} + (1 - f_{min}) \cdot h/100 & h < 60 \end{cases}$$

With $f_{min}$ = 0.5: at h = 59 the factor is 0.795, at h = 35 it is 0.675, at h = 0 it is 0.5.

**Note the discontinuity:** at h = 60 the factor is 1.0, but just below 60 it jumps to 0.8. A real machine would slow gradually. It is harmless for decisions, but it is a modelling simplification.

In a 1-hour balanced shift, A loses about 2 % health (20 units × 0.1), so DEGRADED only appears in scenarios that **start** worn (`fault_burst`: health 60, `section_breakdown`: B at 35).

A **repair restores health to 100** ("repaired and serviced"). This produced a real surprise in WP2: the test "worn machines fail more often" failed, because a worn machine is only worn until its first fault. The test was rewritten to measure time to the *first* fault, and the ratio matches the expected 2.8×.

---

## 6. Random faults: a Poisson process with wear-dependent hazard

### 6.1 Concept: hazard rate and the exponential distribution

If failures happen at a constant **hazard rate** λ (per second), the number of failures in time t is Poisson-distributed and the time between failures is exponential with mean 1/λ (the **MTBF**). The probability of at least one failure in a step Δt is:

$$P(\text{fault in } \Delta t) = 1 - e^{-\lambda \Delta t}$$

This formula is exact for any step size, unlike the approximation λΔt, which can exceed 1.

### 6.2 In the code

```python
hazard_per_s = p.faults_per_hour / 3600.0 * (1.0 + p.fault_wear_gain * (1.0 - self.health / 100.0))
if hazard_per_s > 0.0 and self.rng.random() < 1.0 - math.exp(-hazard_per_s * dt):
    self.faults_total += 1
    self.repair_remaining_s = _gamma(self.rng, p.repair_minutes * 60.0, p.repair_cv)
    return
```

The hazard multiplier is $1 + 3(1 - h/100)$: ×1 when healthy, ×2.2 at health 60, ×4 at health 0. That makes it a **proportional hazards** model with health as the covariate.

Probability of at least one fault in one hour:

| Case | λ per hour | P(≥ 1 fault in 1 h) |
|---|---|---|
| balanced, health 100 | 0.1 | 9.5 % |
| health 60 | 0.22 | 19.7 % |
| `fault_burst` (1.0/h, health 60) | 2.2 | 88.9 % |
| `section_breakdown` B (2.0/h, health 35) | 5.9 | 99.7 % |

(Each probability is the first-fault chance at a constant health level.)

Repair time is Gamma with mean 20 min and CV 0.3. During repair the step consumes repair time first. If the repair finishes mid-step, health resets and production continues with the remaining time.

**Modelling details worth knowing:**
- The fault check happens once per `step()` call **before** production, so a fault can occur while BLOCKED (a stopped machine breaking is questionable but harmless).
- A step that starts a fault ends immediately (`return`), so the rest of that step is not simulated. With 1 s or 0.1 s steps this is negligible.

---

## 7. Time to full: the most useful signal for the AI

```python
def time_to_full_sim_s(self):
    free = capacity - buffer
    if free <= 0:                     return 0.0
    rate = rate_per_hour * rate_factor()
    if status == FAULT or rate <= 0:  return inf
    units_of_work = max(0, free - 1 + (self._work_needed - self._progress))
    return units_of_work / (rate / 3600) / time_scale
```

It counts the **remaining work of the unit currently on the machine** (exactly known, because `_work_needed` for the current unit was already sampled) plus `free − 1` more units at the **mean** work of 1. The test `test_time_to_full_matches_what_actually_happens` confirms the predicted value matches actual blocking time (±0.5 s in the noise-free case).

**Example:** B at t = 0 (seed 1) has free = 8. The current unit needs 0.82 more work, so the estimate is (7 + 0.82) × 120 s = **938 s**.

This value drives:
- priority tier 1 in the symbolic layer (fills within 180 s or within travel time → urgent),
- the rule-based policy's urgency and the fallback's ordering,
- features `target_time_to_full` and `worst_other_time_to_full`,
- the PPO observation.

**A subtle information leak:** this estimate uses the *already sampled* random work of the current unit, which is information a real machine controller would not have exactly. It would only have an estimate from the cycle timer. The effect is small (one unit's variation, ±15 %).

---

## 8. Pickup

```python
def pickup(self, max_units=None, max_mass_kg=None):
    take = self.buffer
    if max_units is not None:   take = min(take, max(0, int(max_units)))
    if max_mass_kg is not None: take = min(take, floor(max_mass_kg / unit_mass_kg + 1e-9))
    self.buffer -= take; self.picked_total += take
    return take
```

The `+ 1e-9` guards against floating-point error: 4.0/0.5 might be 7.9999999 and floor to 7. Picking up immediately unblocks the line, because the status is recomputed from the buffer.

Example from the tests: a buffer of 15 × 0.4 kg with a 5 kg limit gives **12 taken**.

---

## 9. Reproducible randomness: one stream per section

```python
self.rng = random.Random(f"{self.seed}:{self.section_id}")
```

- **Same seed → the same shift**, which is essential for **paired comparison** of policies (file 13). Every policy faces exactly the same random factory *as long as its actions do not change the random draws*.
- **Separate streams per section** mean A and B do not get identical noise, and adding a draw in A does not shift B's sequence.
- **Caveat:** the order of draws inside one section depends on the robot's actions. A pickup that unblocks a line early lets it produce more units, which consumes more Gamma draws. So two policies facing the same seed see the same factory **only until their actions diverge**. Then the fault timings and cycle times differ. This is unavoidable with a single stream per section. A fully "common random numbers" design would pre-sample a separate stream for each random quantity (cycle work, fault arrivals, repair durations).

---

## 10. Time scale

`time.time_scale` converts sim seconds to factory seconds: `dt_factory = dt_sim × time_scale`. All robot-facing values (`time_to_full_s`, `fault_remaining_s`, `blocked_time_s`) are converted back to sim seconds. Rates stay "per factory hour".

It is **1.0 now**. At first it was 20, then 5. The robot moves in real sim time while production was sped up, which made the factory hopelessly overloaded (every line blocked within about 90 s at ts 20). HANDOVER WP2 contains the full capacity calculation that led to 5, and the revision set realistic numbers with ts 1.0.

---

## 11. Configuration loading, merging and typo protection

```python
def load_config(scenario="balanced", config_dir=None, overrides=None):
    base = yaml.safe_load(params.yaml)
    scen = yaml.safe_load(scenarios/<scenario>.yaml)          # unknown scenario -> ValueError listing known ones
    meta = pop "name", "description" from scen
    check_keys(base, scen); check_keys(base, overrides)       # unknown key -> ValueError
    cfg = deep_merge(deep_merge(base, scen), overrides)
    cfg["name"], cfg["description"] = meta ...
```

- **`deep_merge`** recursively merges dicts: a scenario that sets only `factory.sections.B.rate_per_hour` changes only that leaf (verified by `test_scenario_overrides_only_what_it_names`).
- **`check_keys`** walks the override and requires every key to exist in `params.yaml`. For example, `unknown parameter 'params.robot.battery.capacity_w'`.
- **Exception:** under `factory.sections.<id>` any `section_defaults` key is allowed, so `section_breakdown` can give only B a bad machine.

**Code smell:** `_SECTION_DEFAULT_KEYS` is a **module-level global dict** that `load_config` clears and refills, and `check_keys` reads. It works in a single thread, but it is hidden shared state. Passing the allowed keys as an argument would be cleaner and thread-safe.

---

## 12. The 12 scenarios

| Scenario | Overrides | Description (from the file) |
|---|---|---|
| `balanced` | none | Nominal rates, rare faults, healthy fully charged robot |
| `high_demand` | A 40, B 60, C 10 units/h | All lines at 2× nominal |
| `one_hot_section` | B 60 units/h | B blocks within 10 min if neglected |
| `fault_burst` | all: health 60, 1 fault/h, 15 min repair | Lines stop and restart unpredictably |
| `section_breakdown` | B only: health 35, 2 faults/h, 25 min repair | One failing machine |
| `low_battery_start` | battery 30 %; buffers A 4, B 5, C 1 | Low battery while buffers are half full |
| `worn_robot` | robot condition 45 % | Draws more energy, close to the maintenance limit |
| `aged_battery` | 12 Wh pack, 15 W charger | Charging must be planned |
| `heavy_parts` | unit mass ~2×, smaller buffers (5/6/2) | Fewer units per trip |
| `heavy_load` | 2× rates, heavier parts, smaller buffers, battery 50 % | Beyond one robot's capacity on purpose |
| `small_buffers` | buffers 3 / 4 / 1 | Lines block within minutes |
| `hot_factory` | ambient 38 °C, cooling 0.0008 | Long heavy trips risk the motor limit |

The first six existed when the neuro-symbolic model and PPO were trained. **The last six were added afterwards, and neither model was retrained** (the model metadata lists 6 scenarios). They are therefore a real **generalisation test** (file 13).

---

## 13. Capacity analysis: `param_report.py`

### 13.1 What it computes

For each section, assume dedicated full-load round trips delivery → section → delivery:

$$\text{trips/h} = \frac{\text{rate} \times \text{unit mass}}{\min(\text{full buffer mass}, \text{payload})}$$

$$\text{busy fraction} = \frac{\sum_s \text{trips/h}_s \cdot (\text{2 × drive time} + \text{load} + \text{unload})}{3600}$$

$$\text{charge fraction} = \frac{\text{drive energy per hour} + 6\text{ W} \times 1\text{ h}}{P_{net}} \quad\text{(hours of charging per hour)}$$

$$\text{utilisation} = \text{busy} + \text{charge}$$

It also runs 5 seeds of a shift **with no robot** to report when each line first blocks. It warns if a full buffer exceeds the payload, if a single mission from full battery breaks the reserve, if a full trip overheats, or if utilisation > 1.

### 13.2 Current output for all 12 scenarios (run on 2026-09-17)

| Scenario | Utilisation | Busy % | Charge % | kg/h | First BLOCKED A/B/C, no robot (min) | Warnings |
|---|---|---|---|---|---|---|
| aged_battery | 0.99 | 21.4 | 77.5 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |
| balanced | 0.58 | 21.4 | 36.7 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |
| fault_burst | 0.58 | 21.4 | 36.7 | 29.5 | 35.0 / 28.0 / never | 0 |
| heavy_load | **1.03** | 56.9 | 45.8 | 88.0 | 7.6 / 6.0 / 11.9 | 1 (utilisation > 1) |
| heavy_parts | 0.75 | 34.4 | 40.4 | 56.5 | 11.6 / 8.0 / 24.5 | 0 |
| high_demand | 0.85 | 42.8 | 41.9 | 59.0 | 10.6 / 7.9 / 18.1 | 0 |
| hot_factory | 0.58 | 21.4 | 36.7 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |
| low_battery_start | 0.58 | 21.4 | 36.7 | 29.5 | 11.6 / 10.0 / 24.5 | 0 |
| one_hot_section | 0.70 | 30.8 | 39.1 | 41.5 | 23.7 / 7.9 / 36.0 | 0 |
| section_breakdown | 0.58 | 21.4 | 36.7 | 29.5 | 23.7 / never / 36.0 | 0 |
| small_buffers | 0.99 | 56.9 | 42.5 | 29.5 | 5.7 / 6.1 / 12.2 | 0 |
| worn_robot | 0.59 | 21.4 | 37.6 | 29.5 | 23.7 / 15.7 / 36.0 | 0 |

### 13.3 How to interpret it, and its limits

- It is a **steady-state, many-hour** estimate. The **evaluated shift is only 1 hour, starting from the scenario's initial battery**. In `aged_battery` the 12 Wh pack at 100 % already covers about 1.5 h of the 7 Wh/h demand, so utilisation 0.99 overstates the difficulty of a 1-hour shift. The evaluation confirms this: every policy scores about the same as in balanced (file 13). **Energy strategy only becomes critical in shifts longer than one battery charge, or with low starting charge.**
- It assumes single-section round trips. Combined trips (A then B then deliver) make the real busy time lower.
- "Charging" assumes charging happens only when needed. The robot can also wait at the charger, which charges for free.
- `small_buffers` shows the other extreme: utilisation 0.99 from **busy time**, because buffers are so small the robot must make many small trips. Here routing and timing decisions matter most.

Rules of thumb printed by the tool: < 0.5 easy, 0.6–0.9 decisions matter, > 1 production will be lost with any policy.

---

## 14. The live node (`factory_node.py`)

- One process for all three sections (the CPU reason is in file 02 §7).
- A timer at `step_period` = 0.1 **sim** seconds steps every section by `now − last` (it ignores the first tick and clock resets).
- A timer at 1 Hz publishes `SectionStatus` per section and writes one CSV row per section (`logs/<run_id>_section_<id>.csv`).
- The service `/factory/<id>/pickup` maps −1 / ≤ 0 to "no limit" and responds with units, mass, buffer left and a message.
- `seed` parameter −1 means the scenario's seed (`time.seed` = 1).

`factory_monitor` prints a table every second: status, buffer, fill %, produced, picked, actual rate, health, time to full, fault time left, lost.

---

## 15. Key concepts of this section

- Manufacturing line modelling: throughput, buffers, **blocking**, lost production
- Gamma distribution via mean and CV; coefficient of variation as a variability measure
- Discrete-event updates inside a fixed time step → step-size independence
- Hazard rate, Poisson process, exponential inter-arrival, MTBF / MTTR, proportional hazards
- Degradation affecting speed, maintenance restoring it
- Reproducible pseudo-random streams, seeding per component, the limits of common random numbers
- Deep-merge configuration with strict schema validation
- Queueing intuition: utilisation ρ = demand / capacity; above ~0.8 waiting and loss grow sharply

---

## 16. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Longer shifts (e.g. 8 h) in evaluation** | 1-hour shifts hide the energy strategy (a full battery covers most scenarios), which is half of "resource-efficient" | Evaluate with `--shift-s 28800`; energy, charging level and battery ageing then really matter |
| **Pre-sampled random streams** (true common random numbers) | Policies see different factories once their actions diverge, which adds noise to paired comparisons | Pre-generate per section a list of cycle works, fault arrival times (in machine-running time) and repair durations from separate `numpy.random.Generator` streams |
| **Use a DES library** | Growing the model (operators, shared resources, multiple robots) gets hard by hand | `SimPy` (process-based discrete-event simulation) or `salabim`; keep the same `snapshot()` interface |
| **Continuous degradation curve** | The step at health 60 % is artificial | e.g. `rate_factor = f_min + (1 - f_min) * sigmoid((h - 60)/10)` |
| **Upstream starvation and product mix** | Real lines also stop for missing input; different products have different priorities | Add an input buffer fed by the robot (a two-way transport problem) and per-product value in the objective |
| **Uncertain time-to-full** | The estimate uses the true sampled work of the current unit | Publish mean ± std (from CV) so the AI can reason about risk, and use a quantile in the urgency rule |
| **Real data** | Parameters are plausible, not measured | Fit Gamma cycle times and Weibull failure times to MES/PLC logs of a real line (`scipy.stats.gamma.fit`, `lifelines` for survival analysis) |
| **Remove the global `_SECTION_DEFAULT_KEYS`** | Hidden shared state | Pass the allowed section keys into `check_keys` explicitly |
