# 05 — Robot energy, battery, thermal and wear model

File: `src/robofetch_core/robofetch_core/robot_model.py` (263 lines, pure Python, no ROS).
Numbers: `params.yaml → robot:` (and scenario overrides).
Used by `robot_state_node` (live), `FactorySim` (training/evaluation), `mission_plan.predict`, `SymbolicLayer` (safety rules), `features.py` (neural features), `rule_based.py`, `fallback_policy.py`, and `param_report.py`.

This model is the **physics the AI reasons about**. The hard safety rules are only as good as this model, and resource-efficient decisions are only meaningful if these energy numbers are realistic. It is worth understanding completely.

---

## 1. Four coupled quantities

```
             ┌──────────────── payload, distance, motor load ───────────────┐
             ▼                                                              ▼
        ENERGY drawn ───────► BATTERY %                              TEMPERATURE °C
        (drive + idle)         (− drawn, + charged)                   (heating − cooling)
             ▲                                                              │
             │   efficiency penalty (hotter & more worn = more Wh/m)        │
             └──────────────── CONDITION % ◄──── wear per Wh + time above warn_c
```

The feedback loops:
- A **hotter** motor is less efficient, so each metre costs more energy.
- A **more worn** drive draws more energy.
- **More energy** throughput wears the drive.
- **Overheating** wears the drive faster.

---

## 2. Parameters (`RobotParams`)

`RobotParams` is a **frozen dataclass with no defaults**. Every field must come from `params.yaml`. `from_config()` walks the nested YAML through `_CONFIG_MAP` (flat field name → YAML path) and enforces two things:

1. **Missing key → `KeyError`** ("params.yaml is missing robot.battery.capacity_wh").
2. **Unused key → `KeyError`** ("params.yaml has robot parameters no code reads: robot.battery.capacity_w"). This catches typos in the base file and dead parameters.

Values for the balanced scenario and where they come from (comments in `params.yaml`):

| Group | Field | Value | Justification |
|---|---|---|---|
| — | `mass_kg` | 4.3 | URDF: chassis 3.0 + wheels 1.0 + casters/lidar 0.3 |
| battery | `capacity_wh` | 22.0 | 11.1 V × 2.0 Ah Li-ion (TurtleBot3 class: 19.98 Wh) |
| battery | `reserve_percent` | 15 | must remain **after** returning to the charger |
| battery | `charge_power_w` | 25 | charger output (~1.1 C) |
| battery | `initial_percent` | 100 | |
| energy | `idle_power_w` | 6.0 | single-board computer ~4 W + lidar ~1.5 W + drivers ~0.5 W, drawn always |
| energy | `drive_wh_per_m` | 0.005 | ×0.39 m/s ≈ 7 W motor power |
| energy | `load_wh_per_m_per_kg` | 0.00116 | = drive_wh_per_m / mass_kg (rolling resistance ∝ total mass) |
| energy | `temp_loss_per_c` | 0.004 | efficiency loss per °C above 25 °C |
| energy | `worn_extra_draw` | 0.30 | fully worn drive draws +30 % |
| thermal | `ambient_c` | 22 | |
| thermal | `heat_c_per_s` | 0.037 | heating at full load |
| thermal | `cool_per_s` | 0.0011 | Newton cooling coefficient |
| thermal | `payload_heat_per_kg` | 0.05 | 5 kg → +25 % heating |
| thermal | `dock_cooling_factor` | 1.5 | parked cools faster |
| thermal | `warn_c` / `max_c` / `resume_c` | 55 / 70 / 45 | wear accelerates / must stop / may resume |
| wear | `condition_min_percent` | 30 | below this: maintenance, no work |
| wear | `per_wh` | 0.01 | condition % lost per Wh drawn |
| wear | `per_c_s_above_warn` | 0.0005 | % per second per °C above warn |
| motion | `speed_m_s` | 0.39 | **measured** maze average including turns |
| motion | `drive_load` | 0.7 | typical motor load while driving |
| motion | `dwell_load` | 0.0 | motors off while loading/waiting |
| handling | `max_payload_kg` | 5.0 | |
| handling | `load_time_s` / `unload_time_s` | 30 / 30 | |

**History (HANDOVER WP2 revision):** the first values were unrealistic. Drive energy was 0.35 Wh/m, which implies about 490 W, 70× too high. Charging was 1 %/s, which is full in 100 s. Heating was 1.2 °C/s, which is 70 °C within a minute. The first "corrected" drive value, 0.012 Wh/m, was still 3.6× off, and was caught by the test `test_realistic_magnitudes` (5–40 Wh/km, 1–4 h driving runtime, 30–120 min charge).

---

## 3. Energy equations

### 3.1 Efficiency penalty (≥ 1)

$$\eta_{pen}(T, c) = \underbrace{\big(1 + k_T \cdot \max(0, T - 25)\big)}_{\text{thermal}} \cdot \underbrace{\big(1 + k_w \cdot \max(0, 1 - c/100)\big)}_{\text{wear}}$$

with $k_T$ = `temp_loss_per_c` = 0.004 and $k_w$ = `worn_extra_draw` = 0.30. `T_REF_C = 25` is a definition, not a tunable.

Examples:
- New robot at 25 °C: 1.0 × 1.0 = **1.0**
- `worn_robot` scenario (condition 45 %) at 25 °C: 1 × (1 + 0.3 × 0.55) = **1.165**
- 55 °C and condition 45 %: (1 + 0.004 × 30) × 1.165 = 1.12 × 1.165 = **1.305**

### 3.2 Drive energy (motors only)

$$E_{drive}(d, m) = d \cdot \big(e_0 + e_m \cdot m\big) \cdot \eta_{pen}$$

with $e_0$ = 0.005 Wh/m and $e_m$ = 0.00116 Wh/(m·kg).

- Empty: **0.005 Wh/m**. With 5 kg: 0.005 + 0.0058 = **0.0108 Wh/m** (2.16×).
- 10 m with 4 kg: 10 × (0.005 + 0.00464) = **0.0964 Wh**.

### 3.3 Idle energy (electronics)

$$E_{idle}(t) = P_{idle} \cdot t / 3600, \qquad P_{idle} = 6\ \text{W}$$

This is drawn **all the time**: driving, loading, waiting, but not while docked (the charger powers the electronics, §4).

### 3.4 Trip energy (what the AI uses)

$$E_{trip}(d, m) = E_{drive}(d, m) + E_{idle}(d / v)$$

- 10 m with 4 kg: 0.0964 + 6 × (10/0.39)/3600 = 0.0964 + 0.0427 = **0.139 Wh**
- B → delivery (10.26 m) with 4 kg: **0.143 Wh = 0.65 % of the battery**
- charger → B (15.18 m) empty: drive 0.0759 + idle 0.0649 = **0.141 Wh**, 38.9 s

### 3.5 The key insight from these numbers

From `param_report.py` (HANDOVER):
```
driving 13.0 W total at 0.39 m/s   (6 W idle + 7 W motors)
energy per km (empty, incl. idle) 9.3 Wh; range empty 2372 m, with 5 kg 1460 m
runtime: driving non-stop 1.69 h, standing idle 3.67 h; charging reserve -> full 59 min
FULL-LOAD TRIP delivery -> section -> delivery (incl. 60 s handling)
   A  10.01 m  4.00 kg  111 s  0.33 Wh  1.5 %
DEMAND: 29.5 kg/h; robot busy 21.4 %, energy 7.0 Wh/h -> charging 36.7 %  => UTILISATION 0.58
```

> **With realistic numbers, most of the robot's energy is the electronics' idle power (about 6 of 7 Wh per hour in `balanced`). One full-load trip costs only about 1.5 % of the battery.**

This shapes the whole AI problem:
- **Energy efficiency is mostly about time**: fewer wasted minutes, not fewer metres. Waiting at the charger is free (docked), while waiting in an aisle costs 6 W.
- **Charging is slow** (about 1 hour from reserve), so *when* and *how much* to charge is a real strategic decision. It is why `CHARGE:40/70/100` are separate actions.
- The objective's `cost_per_wh: 0.5` makes 1 Wh worth half a delivered unit. A trip of about 0.3 Wh costs about 0.15 units of value, so the objective is dominated by delivered and lost units, not energy (file 08).

---

## 4. Battery and charging

```python
def net_charge_power_w(p):
    return p.charge_power_w - p.idle_power_w            # 25 - 6 = 19 W into the pack

def charge_time_s(p, from_percent, to_percent=100.0):
    need_wh = max(0, to - from) / 100 * capacity_wh
    return inf if net_power <= 0 else need_wh / net_power * 3600
```

**Concept: net charging power.** While docked the charger also runs the computer, so only 19 W reaches the battery. This is a **constant-power** model, with no CC/CV taper. Real Li-ion chargers slow down above ~80 % state of charge (file 16).

Examples:
- 15 % → 100 %: 0.85 × 22 = 18.7 Wh / 19 W = 0.984 h = **3543 s ≈ 59 min**
- 15 % for 600 s docked: 19 × 600/3600 = 3.17 Wh = +14.4 %, which gives **29.4 %**
- 60 % → 100 % (CHARGE:100 from delivery): 5.1 m drive + 8.85 Wh charge ≈ **1689 s ≈ 28 min**

Measured in Gazebo (WP3): charging 27.1 → 40 % in about 550 s = 2.84 Wh ≈ **18.6 W**, against the configured 19 W net.

---

## 5. Thermal model

In `RobotCondition.step`:

```python
cooling = p.cool_per_s * (p.dock_cooling_factor if docked else 1.0)
heating = 0.0 if docked else p.heat_c_per_s * motor_load * (1.0 + p.payload_heat_per_kg * payload_kg)
self.temperature_c += dt * (heating - cooling * (self.temperature_c - p.ambient_c))
self.temperature_c = max(p.ambient_c, self.temperature_c)
```

As a differential equation (explicit Euler integration with step dt):

$$\frac{dT}{dt} = h \cdot L \cdot (1 + k_p m) - c\,(T - T_{amb})$$

**Concept: first-order system / Newton's law of cooling.** With constant input the solution approaches a **steady state** exponentially:

$$T_{ss} = T_{amb} + \frac{h L (1 + k_p m)}{c}, \qquad \tau = \frac{1}{c}$$

| Case | Steady state | Time constant |
|---|---|---|
| balanced, load 1.0, empty | 22 + 0.037/0.0011 = **55.6 °C** | 909 s ≈ **15 min** |
| balanced, load 0.7 (normal driving), empty | 22 + 23.5 = **45.5 °C** | 15 min |
| balanced, load 1.0, 5 kg | 22 + 42.0 = **64.0 °C** | 15 min |
| `hot_factory` (ambient 38, cool 0.0008), load 0.7, 5 kg | 38 + 40.5 = **78.5 °C** (above max 70!) | 1250 s ≈ **21 min** |
| `hot_factory`, load 1.0, 5 kg | **95.8 °C** | 21 min |

Simulated: 600 s of continuous driving with 5 kg at load 0.7 in balanced gives **36.2 °C** (still far from steady state), battery 100 → 83.7 %, energy 3.58 Wh, 234 m.

**Why the heat limit rarely matters in practice:** trips last about 20–70 s, then the robot stands still for 30 s of loading with no heating. Temperature therefore stays well below the steady state. Only `hot_factory` with long heavy continuous work gets near 70 °C, which is why that scenario exists.

**Euler accuracy:** with dt = 1 s and c = 0.0011, dt·c ≪ 1, so explicit Euler is essentially exact. With a much larger dt (e.g. 1000 s) it would overshoot.

---

## 6. Wear (condition) model

```python
overheat = max(0.0, self.temperature_c - p.warn_c)
self.condition_percent = max(0.0, self.condition_percent - (
    p.wear_per_wh * used + p.wear_per_c_s_above_warn * overheat * dt))
```

$$\Delta c = -\big(w_E \cdot E_{used} + w_T \cdot \max(0, T - T_{warn}) \cdot dt\big)$$

- Energy wear: 0.01 % per Wh. About 10 Wh per shift gives about 0.1 % per shift, so a new drive would wear out after roughly 1000 shifts, as the comment says.
- Heat wear: 0.0005 % per (°C·s) above 55 °C. One hour at 63 °C (8 °C over) costs 0.0005 × 8 × 3600 = **14.4 %**.

In a 1-hour shift wear is negligible **unless the robot starts worn** (`worn_robot`: 45 %). Then the effect is on energy through the 1.165× penalty, and on the maintenance rule (below 30 % the robot may only charge or wait, hard rule H1 in file 10).

---

## 7. `RobotCondition`: the integrator

A mutable dataclass holding `battery_percent`, `temperature_c`, `condition_percent`, plus cumulative distance, energy drawn, energy charged, uptime and overheat time.

`step(dt, distance_m, payload_kg, motor_load, docked)` advances everything by `dt`:

```
if docked:
    add min(net_charge_power × dt, room left) to the battery          (no energy "used")
else:
    used = drive_energy(distance, payload, current T, current condition) + idle_energy(dt)
    battery -= used (floored at 0); cumulative energy += used; distance += distance
temperature update (heating 0 when docked, cooling × 1.5 when docked)
wear update (uses `used`, which is 0 when docked)
```

Two subtle points:
1. **The order matters**: energy is computed with the temperature *before* this step's heating.
2. **Docked means "on the charger AND charging".** The fast simulator passes `docked=True` for CHARGE, and for WAIT only at the charger. The live `robot_state_node` sets `docked = (activity == "charging")`, and the executor sets activity "charging" for CHARGE and for WAIT at the charger. Both sides agree.

`copy()` makes an independent copy (params shared, since they are immutable), used for look-ahead.

---

## 8. `simulate_route`: forward simulation for safety checks

```python
def simulate_route(condition, legs, dt=1.0, dwell_s=0.0):
    sim = condition.copy()                  # never touches the real state
    for distance_m, payload_kg in legs:
        walk(travel_time_s(p, distance_m), distance_m, payload_kg, p.drive_load)
    if dwell_s > 0:
        walk(dwell_s, 0.0, 0.0, p.dwell_load)
    return sim, peak_temperature
```

**Why simulate instead of using the closed-form `trip_energy_wh`?** Because temperature is a *trajectory*. The peak can exceed the limit mid-route even if the end state is fine, and energy depends on the temperature along the way. The test `test_simulated_route_energy_matches_the_closed_form` confirms both agree when temperature effects are small.

**Example (used by hard rule H3, file 10):** the robot is at the charger with 30 % battery and considers PICKUP:B. The route is charger → B (15.18 m, empty), then B → charger (15.18 m), with 30 s dwell. The result is an end battery of **28.49 %** and a peak of 23.9 °C. The rule requires reserve 15 + safety margin 2 = 17 %, and 28.49 ≥ 17, so it is **allowed**.

**Note on the dwell:** `walk(dwell_s, 0, 0, dwell_load)` passes payload 0 during the dwell. That only affects heating, which is 0 anyway with `dwell_load = 0`.

---

## 9. `mission_plan.predict()`: cost prediction per action

`predict(p, matrix, action, location, battery, payload, temperature, condition)` returns a `Prediction(distance, duration, energy, charged, battery_end)`. It uses the closed-form functions, not step integration:

| Action | Distance | Duration | Energy drawn | Charged |
|---|---|---|---|---|
| `PICKUP:X` | matrix[loc][X] | d/v + load_time | trip(d, payload) + idle(load_time) | 0 |
| `DELIVER` | matrix[loc][delivery] | d/v + unload_time | trip(d, payload) + idle(unload_time) | 0 |
| `CHARGE:p` | matrix[loc][charger] | d/v + charge_time(battery_after_drive → p) | trip(d, payload) | (p − battery_after_drive) % of capacity |
| `WAIT:s` at charger | 0 | s | 0 | min(19 W × s, room) |
| `WAIT:s` elsewhere | 0 | s | idle(s) | 0 |

Real examples (balanced, 25 °C, condition 100):

| Action from | Predicted distance | Duration | Energy | Battery end |
|---|---|---|---|---|
| PICKUP:B from charger, 100 %, empty | 15.18 m | 68.9 s | 0.191 Wh | 99.13 % |
| DELIVER from B, 99 %, 4 kg | 10.26 m | 56.3 s | 0.193 Wh | 98.12 % |
| CHARGE:100 from delivery, 60 % | 5.10 m | 1689 s | 0.047 Wh | 100 % (8.85 Wh charged) |

Measured vs predicted in Gazebo (HANDOVER WP3, 12-action low-battery mission): duration −2.1 %, distance +0.2 %, energy −0.9 %.

**Small inconsistency:** `predict()` has default `temperature_c=25.0`, while `RobotCondition` starts at `ambient_c` = 22. The executor always passes telemetry values, so live predictions are not affected.

---

## 10. `robot_state_node`: the live integrator

The node subscribes to `/odom` and `/robot/activity`, publishes `/robot/telemetry` (JSON, 1 Hz), and writes `logs/<run_id>_robot.csv`.

- **Distance from odometry, not from commanded velocity** (docstring): if the robot pushes against a wall, the wheels turn but it does not move, and a command-based model would count energy for travel that never happened. Odometry is a better proxy. (Wheel slip would still fool odometry; ground truth is not used because a real robot has none.)
- **Motor load estimate:** `load = min(1, speed / speed_m_s × drive_load)`. At 0.39 m/s this gives 0.7, and at the 0.5 m/s peak 0.9. The fast simulator uses a constant 0.7 while driving.
- **Sim clock integration:** `dt = now − last_tick` in sim time. It skips the first tick and clock resets (`now <= last`).
- **Duty state** (`_duty_state`), evaluated in priority order: `stopped` (e-stop), then `cooldown` (T ≥ 70), then `fault` (condition < 30), then `charging`, then `cooldown` (idle and above 55 °C), then the activity text. It is published for monitoring only. **Nothing in the decision layer enforces a cooldown**. The fast simulator records `overheated` as a violation instead, and the symbolic rule prevents routes that would reach 70 °C.
- The CSV is flushed every tick, so a killed launch does not lose the run.

**Stale docstring content:** it mentions "tools/ml/", "task manager", "admission control" and `order_id` from the old RoboFetch project. The comment "Warn once per crossing" is also wrong: the code warns **every tick** while below the reserve or overheated.

---

## 11. Key concepts of this section

- Energy = power × time; idle vs motion power decomposition
- Specific energy consumption (Wh/m, Wh/km) and payload scaling ∝ total mass
- Multiplicative efficiency penalties
- Constant-power charging with a parasitic load (net power)
- First-order thermal model, steady state, time constant, explicit Euler integration
- Degradation models: throughput-based wear + stress-based (thermal) wear
- Forward simulation for constraint checking (peak vs end state)
- Model-based prediction vs measurement as a validation loop

---

## 12. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **CC/CV charging curve** | Real Li-ion charging slows strongly above ~80 % SoC. The model makes the last 30 % as fast as the first, so "charge to 100 %" looks cheaper than it really is | Piecewise: constant power until 80 %, then exponential taper `P(soc) = P_max · exp(-(soc-0.8)/τ)`. The CHARGE actions and look-ahead would automatically value partial charges more |
| **Battery equivalent circuit / Peukert effect** | Capacity depends on discharge current and temperature; voltage sag at low SoC | A Thevenin 1-RC model or a lookup of usable capacity vs current (`PyBaMM` for a physics-based Li-ion model if the thesis wants depth) |
| **Battery ageing (cycle and calendar)** | `aged_battery` is a static scenario; real capacity fades with cycles and deep discharges, which is a real *resource-efficiency* objective | Add capacity fade ∝ throughput and depth-of-discharge (e.g. a rainflow-counting cycle model); penalise deep discharge in the objective |
| **Turning and acceleration energy** | Energy is distance-only; stop-and-go and spinning in place cost real energy and differ between planners (WP8) | Add `wh_per_rad` and an acceleration term; calibrate with Gazebo joint torques × wheel speeds |
| **Calibrate parameters from data (system identification)** | Parameters are literature estimates; the Gazebo robot has no real motor model | Fit `drive_wh_per_m`, heating/cooling from logged runs with `scipy.optimize.least_squares`, or Bayesian calibration (`PyMC`) to get uncertainty bands for the safety margin |
| **Uncertainty-aware safety margin** | H3 uses a fixed 2 % margin on a deterministic prediction | Use the measured prediction-error distribution (≈ ±2 %) and choose the margin as a high quantile; conformal prediction gives distribution-free bounds |
| **Enforce cooldown in the executor** | `cooldown` is only reported | If telemetry state = cooldown, force WAIT at the current place (or CHARGE) until `resume_c`, the same as a real motor-protection controller |
| **Fix the stale docstrings/comments** | Misleading for readers | Update `robot_state_node.py` docstring and the "warn once" comment (or implement edge-triggered warnings) |
