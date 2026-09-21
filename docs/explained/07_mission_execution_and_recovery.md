# 07 — Mission execution, navigation recovery and logging

Files:
- `src/robofetch_core/robofetch_core/mission_plan.py`: action vocabulary, parsing, `predict()`
- `src/robofetch_core/robofetch_core/mission_executor.py` (723 lines): the node that runs everything
- `src/robofetch_core/robofetch_core/fallback_policy.py`: decisions without the AI (details in file 09)
- `src/robofetch_interfaces/msg/TaskEvent.msg`
- `scripts/run_mission.sh`, `scripts/obstacle.sh`, `scripts/test_nav_recovery.sh`
- `params.yaml → mission.navigation`

The executor is the bridge between **decision** ("PICKUP:B") and **physical execution** (Nav2 goals, dwell times, service calls). It is also where robustness lives: timeouts, false-success detection, recovery and safe halting.

---

## 1. The action vocabulary (`mission_plan.py`)

```python
PICKUP, DELIVER, CHARGE, WAIT = "PICKUP", "DELIVER", "CHARGE", "WAIT"
SECTIONS = ("A", "B", "C")
DELIVERY, CHARGER = "delivery", "charger"

@dataclass(frozen=True)
class Action:
    kind: str
    target: str = ""      # section for PICKUP
    value: float = 0.0    # battery % for CHARGE, seconds for WAIT

    def destination(self):
        return {PICKUP: self.target, DELIVER: DELIVERY, CHARGE: CHARGER}.get(self.kind)   # WAIT -> None
```

| Action | Where it drives | What it does there | Ends when |
|---|---|---|---|
| `PICKUP:A` | POI `A` | stands `load_time_s` (30 s), then takes as many units as still fit the payload | after the service call |
| `DELIVER` | POI `delivery` | stands `unload_time_s` (30 s), empties all cargo | after the dwell |
| `CHARGE:70` | POI `charger` | charges | battery ≥ 70 % |
| `WAIT:60` | nowhere | stays; **at the charger this charges** | after 60 s |

**Why these are good *macro-actions*:**
- One decision covers a whole trip (drive + work), so a 1-hour shift is about 45–65 decisions instead of thousands of velocity commands.
- The robot can **collect from several sections before delivering** (PICKUP:B → PICKUP:A → DELIVER), which is a real routing choice.
- CHARGE has a **target**, so a quick top-up is expressible (added in WP5).

`Action` is `frozen=True`, which makes it **immutable and hashable**. The AI uses actions as dictionary keys (`{action: Verdict}`, `{action: score}`).

### Parsing

`parse_action("PICKUP:b")` → `Action("PICKUP", "B")`. `parse_plan("PICKUP:B; PICKUP:A; DELIVER; CHARGE:90; WAIT:60")` gives a list. Validation errors, all covered by tests: `PICKUP` without a section, `PICKUP:D`, `DELIVER:A`, `CHARGE:0`, `CHARGE:120`, and `WAIT` without positive seconds. `CHARGE` alone means 100 %.

`__str__` produces the same format (`CHARGE:70`, using `:g` formatting), so actions travel as strings over HTTP and back.

---

## 2. Executor structure

### 2.1 Threads and executor model

```
main thread:   rclpy SingleThreadedExecutor.spin()   ← delivers ALL callbacks (subscriptions, futures)
worker thread: MissionExecutor._run()                ← the mission logic, blocks on futures/events
```

The worker never calls `spin`. It waits on futures with `_wait(future, timeout)`:

```python
def _wait(self, future, timeout=300.0):
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return future.result() if done.wait(timeout) else None       # None = timed out
```

**Why:** in rclpy, blocking inside a callback while waiting for another callback deadlocks a single-threaded executor. Moving the mission to its own thread keeps the executor free to deliver the responses. All subscriptions and clients use a `ReentrantCallbackGroup`.

**Stale docstring:** the module docstring says "a MultiThreadedExecutor serves the callbacks", but `main()` uses a `SingleThreadedExecutor` (changed in WP3 because the multi-threaded one used 76 % CPU; `main()`'s own comment explains it).

### 2.2 Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `scenario` | balanced | which `params.yaml` + scenario to load |
| `plan` | "" | scripted actions (used when `model` is empty) |
| `model` | "" | `ns`, `ns_symbolic_only`, `ns_neural_only`, `ppo`, `rule`, or `fallback` |
| `decision_url` | `http://localhost:8001/decide` | decision service |
| `decision_timeout_s` | 5.0 | HTTP timeout, then fallback |
| `shift_duration_s` | 0.0 | 0 → `time.shift_duration_s` from config |
| `run_id`, `log_dir` | generated / `<ws>/logs` | log naming |
| `charge_check_period_s` | 2.0 | how often CHARGE re-checks the battery |

### 2.3 Internal bookkeeping

`location` (starts at `charger`), `payload_kg`, `cargo` (section → units), `delivered`, `seq`, `rows` (finished actions), `stuck`, `nav_stats`, `decision_stats`, and the latest `telemetry` JSON, `sections` messages, `amcl_xy`, and ground-truth `_truth` plus accumulated `distance_travelled`.

---

## 3. Startup sequence (`_run`)

```
1. wait_for_navigation_active()      poll /lifecycle_manager_navigation/is_active (≤ 240 s)
       └─ fails → log "Nav2 navigation never became active ... mission aborted" and return
2. nav_client.wait_for_server()
3. wait until the first /robot/telemetry arrives
4. wait for /factory/A|B|C/pickup services
5. publish_initial_pose()            publish the charger pose on /initialpose every 2 s until /amcl_pose arrives (≤ 120 s)
6. sim_sleep(3.0)                    let AMCL settle
7. shift_start_s = sim_now()
8. autonomous loop (model set) OR scripted loop (plan)
9. write_summary()
```

**Autonomous loop:**

```python
while not abort and not stuck:
    if sim_now() - t_start >= shift_duration_s: break      # shift over
    if not self.sections: sim_sleep(2); continue            # no factory data yet
    action, why = self.next_action()                        # service or fallback (file 14)
    ok_count += self.run_action(action)
```

The shift ends only **between** actions. A CHARGE:100 started at minute 59 can run well past the end. The fast simulator stops exactly at the shift end instead (`FactorySim._advance` sets `done`). This is a small live-vs-sim difference.

---

## 4. Executing one action (`run_action` → `_execute`)

```
run_action(action)
 ├─ seq += 1
 ├─ pred = predict(p, matrix, action, location, battery, payload, temperature, condition)
 ├─ record t0 (sim), distance0 (ground truth), energy counters, battery
 ├─ publish TaskEvent STARTED
 ├─ ok, detail, units = _execute(action)
 │     ├─ PICKUP/DELIVER/CHARGE: activity "driving" → navigate_to(destination)
 │     │      └─ not reached → activity "waiting" → _handle_unreachable() → FAILED
 │     ├─ PICKUP : activity "loading" → sim_sleep(30) → call /factory/X/pickup(max_mass = 5 - payload)
 │     │           payload += mass; cargo[X] += units
 │     ├─ DELIVER: activity "unloading" → sim_sleep(30) → delivered += cargo; cargo = {}; payload = 0
 │     ├─ CHARGE : activity "charging" → loop sim_sleep(2 s) while battery < target
 │     └─ WAIT   : activity "charging" if at charger else "waiting" → sim_sleep(value)
 ├─ time.sleep(1.2)            let the next 1 Hz telemetry sample include the end of the action
 ├─ measured: duration (sim), distance (ground truth), energy drawn/charged (telemetry counters)
 └─ publish TaskEvent SUCCEEDED/FAILED + CSV row
```

**The `/robot/activity` message is essential.** `robot_state_node` has no other way to know the robot is charging (docked) or carrying payload. Its battery model uses `docked = activity == "charging"`. **The battery model trusts the executor**: it charges whenever the activity says "charging", regardless of the physical position. That is fine because CHARGE and WAIT set "charging" only after successfully reaching the charger, or when already there. A real robot would read the charger's contact or current sensor instead.

**Note on PICKUP order:** loading time passes *before* the pickup call, so units finished during those 30 s are included. The fast simulator does the same (advance `load_time_s`, then `section.pickup`), so they are consistent.

---

## 5. Navigation with verification and a recovery ladder

### 5.1 Timeout per drive

$$\text{timeout} = \max\big(\text{timeout\_min\_s},\ \text{timeout\_factor} \times d / v\big) = \max(60,\ 3 \times d / 0.39)$$

Examples: 5.1 m → max(60, 39) = 60 s. 18.28 m → max(60, 140.6) = 141 s. A stuck or oscillating robot is cancelled instead of hanging the mission. Nav2's own progress checker (0.25 m in 30 s) catches some cases, but not an oscillating robot that keeps "moving".

### 5.2 One attempt (`_drive_once`)

1. Send a `NavigateToPose` goal to the POI pose (map frame, yaw → quaternion `(0, 0, sin(yaw/2), cos(yaw/2))`).
2. Rejected → fail "goal rejected by Nav2".
3. Wait for the result up to the timeout. If it times out, cancel the goal and return "timeout after N s" (`nav_stats.timeouts += 1`).
4. E-stop → fail "emergency stop".
5. Status 4 (SUCCEEDED) → **verify arrival**:
   ```python
   time.sleep(0.5)
   error = math.dist(self.amcl_xy, (poi["x"], poi["y"]))
   if error > arrival_tolerance_m (0.35):
       return False, f"Nav2 reported success but the robot is {error:.2f} m from {poi_name} (goal blocked?)"
   ```
6. Other status → fail with "Nav2 aborted"/"canceled".

**Why verify?** (WP3b finding.) When the goal pose is occupied, Nav2's planner (`tolerance: 0.5`) plans to the nearest free spot, the controller reaches it, and Nav2 reports **SUCCESS**. Without verification, a PICKUP would "succeed" next to a blocked section. The tolerance of 0.35 m sits above normal arrivals (≤ 0.22 m measured) and below the goal-shift of a blocked pose.

**Why AMCL and not ground truth?** A real robot has no ground truth. The check must work with what the robot really knows.

### 5.3 The ladder (`navigate_to`)

```
attempt 1 ── fail ──► recovery: clear both costmaps (stale obstacles)
attempt 2 ── fail ──► recovery: BackUp 0.3 m at 0.1 m/s (wedged against something)
attempt 3 ── fail ──► give up
```

Each step is appended to `steps`, which fills the CSV `recovery` column. `nav_stats` counts drives, failed attempts, timeouts, recoveries that worked, abandoned goals and returns to the charger.

After a failed attempt, `self.location = self.nearest_poi()`, the POI closest to the robot's **ground-truth** position. The robot no longer knows where it logically "is", so the next distance lookup uses the nearest POI.

**Inconsistency:** arrival verification deliberately avoids ground truth, but `nearest_poi()` uses it. On a real robot this should use `amcl_xy`.

### 5.4 After giving up (`_handle_unreachable`)

```
destination unreachable
 ├─ emergency stop?                            → report, stop
 ├─ destination IS the charger                 → _declare_stuck(): halt, mission ends "navigation_stuck"
 ├─ return_to_charger_on_failure == false      → just report the failure
 └─ else drive to the charger (full ladder again)
        ├─ reached → "could not reach X; returned to the charger"   (cargo stays on board)
        └─ not reached → _declare_stuck()
```

`_declare_stuck` publishes a zero `Twist` 10 times, sets activity "stopped", logs `NAVIGATION STUCK ... needs a human`, and sets `stuck = True`, which ends the mission loop.

**Design choices:**
- **Cargo is kept.** It is virtual, and a later DELIVER still delivers it.
- **The charger is the safe place.** It is where the robot can wait at no cost and charge.
- **The AI is not told *why* an action failed**, beyond the resulting state (location = charger, same cargo). The next decision simply happens from there. The fast simulator **never models navigation failures**, so no model learned how to react to them (file 16).

### 5.5 Real test results (HANDOVER WP3b)

**Test 1**, a box on C's pickup pose, plan `PICKUP:C;DELIVER`:
```
drive to C failed (Nav2 reported success but the robot is 0.62 m from C (goal blocked?)), attempt 1
reached C after recovery: ['attempt 1: ...0.62 m from C...', 'recovery: clear costmaps']
[1] PICKUP:C done | [2] DELIVER done | ended_by: plan_finished
navigation: drives 2, drive_attempt_failures 1, recovered_after_retry 1, goals_abandoned 0
```
The false success was detected, the recovery worked, and the mission continued. (The robot still loaded about 0.3 m off. The tolerance allows it.)

**Test 2**, a box on the charger after the robot left, plan `PICKUP:A;CHARGE:100`:
```
drive to charger failed (timeout after 70 s), attempt 1
drive to charger failed (Nav2 aborted), attempt 2          (after clear costmaps)
drive to charger failed (timeout after 70 s), attempt 3    (after back up)
NAVIGATION STUCK: the robot cannot reach the charger. Halting - needs a human.
ended_by: navigation_stuck
```

---

## 6. Emergency stop

`ros2 topic pub --once /robot/estop std_msgs/msg/String "{data: stop}"`:

```python
def _on_estop(self, _msg):
    self._abort.set()
    handle.cancel_goal_async()                  # cancel the current Nav2 goal
    for _ in range(10):
        self.halt_pub.publish(Twist()); time.sleep(0.1)     # brake: zero velocity for 1 s
    self._activity("stopped")
```

Every loop checks `_abort`, including `sim_sleep`, the ladder and the CHARGE loop, so the mission ends at the next check. The summary says `ended_by: emergency_stop` and `run_mission.sh` returns 1.

Measured (WP3): stop 12 s into a drive, ground-truth speed **0.771 → 0.001 m/s within 2 s**, then 0.0 for the next 10 s.

This is a **software** stop that depends on ROS messaging. Real industrial safety requires a hardware safety chain (safety-rated stop, PL d / SIL 2) that does not depend on software. The thesis should state that clearly if it claims safety.

---

## 7. Logging and prediction-vs-measurement

### 7.1 TaskEvent + CSV (`logs/<run_id>_mission.csv`)

Columns: `seq, action, target, phase, detail, sim_start_s, duration_s, predicted_duration_s, distance_m, predicted_distance_m, energy_wh, predicted_energy_wh, charged_wh, predicted_charged_wh, battery_start_percent, battery_end_percent, predicted_battery_end_percent, payload_kg, units, location, recovery`

- **Measured distance** comes from ground truth (sum of 5 Hz position deltas).
- **Measured energy** comes from the difference of `cumulative_energy_wh` in telemetry.
- The CSV writes only finished phases and is flushed on every row.

### 7.2 Summary (`logs/<run_id>_mission_summary.yaml`)

```yaml
run_id: run_20260916_170805
scenario: balanced
seed: 1
plan: ''
ended_by: plan_finished          # or emergency_stop / navigation_stuck
actions: 17
succeeded: 17
failed: 0
sim_duration_s: ...
distance_m: 170.1
energy_drawn_wh: 2.505
energy_charged_wh: ...
battery_end_percent: 88.6
delivered_units: {A: 4, B: 8, C: 1}
model: ns
decisions: {asked: 17, answered_by_model: 17, fallback: 0, mean_latency_ms: ...}
navigation: {drives: 17, drive_attempt_failures: 0, timeouts: 0, recovered_after_retry: 0, goals_abandoned: 0, returned_to_charger: 0}
prediction_error_percent (non-charge actions, total measured vs predicted): {duration: -0.5, distance: -0.1, energy: -0.2}
```

(Values from the WP7 run in HANDOVER. The exact file is in `logs/`.)

**The prediction error formula** is aggregate, not a per-action mean:

$$\text{err} = 100 \cdot \frac{\sum_i m_i - \sum_i p_i}{\sum_i p_i}$$

over succeeded, non-CHARGE actions with predicted value > 0. Per-action errors can be larger (WP4 found PICKUP:B at +10 % while the total was 0.6 %).

**Issues in the summary:**
- `ended_by: plan_finished` is also written when an **autonomous shift** ends normally. `shift_over` would be clearer.
- `seed` is `cfg["time"]["seed"]`, not the `seed` launch argument the **factory node** actually used. When `seed:=3` is passed, the summary says 1. `validate_against_gazebo.py` then replays the wrong seed. The actions' costs do not depend on the seed, but the units picked up do.
- `mean_latency_ms` includes the first request, which loads the model (about 1.5 s, noted in HANDOVER).

---

## 8. Scripts for batch and failure testing

- **`run_mission.sh "<plan>" [scenario] [timeout]`**: refuses to start if a simulation is running, launches headless, greps the launch log for `MISSION COMPLETE` / `MISSION ENDED` / `mission aborted` / process death, stops everything it started, and exits 0 = complete, 1 = failed, 2 = timeout, 3 = refused. It is the basis for reproducible Gazebo runs.
- **`obstacle.sh add <name> <poi|x y> [sx sy]`**: spawns a static red 0.6 × 0.6 × 0.5 m box into the running world with `ros_gz_sim create -string <sdf>`. `remove` calls the `/world/factory_maze/remove` gz service.
- **`test_nav_recovery.sh [1|2|all]`**: runs the two failure scenarios end to end. It waits for a trigger line in the launch log, drops the box, and collects the log lines, summary and recovery column into `logs/nav_recovery_<time>.txt`.

---

## 9. Key concepts of this section

- **Macro-actions / options**: temporally extended actions with their own internal control (Sutton, Precup & Singh's options framework)
- **Plan–execute–monitor loop** with prediction vs measurement (model-based monitoring)
- **Timeouts as liveness guarantees**
- **Verification of success signals** (do not trust a subsystem's own success flag when you can cross-check)
- **Recovery ladder / escalation strategy**: cheap fixes first, safe state last
- **Safe state and graceful degradation**: return to charger, halt and ask for a human
- **Threading with futures** in rclpy
- **Structured, append-only run logs** for reproducible experiments

---

## 10. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Tell the AI about failures** | After an abandoned goal the model just sees "at the charger" and may choose the same blocked section again forever | Add `recent_failures` per destination (count, time since) to the state; symbolic rule: forbid a destination for N minutes after `goals_abandoned`; train with simulated failures |
| **Model navigation failures in `FactorySim`** | The AI never experiences blocked routes, so it cannot learn a strategy for them | Random blockage events per corridor/POI (probability + duration), drive time distribution from Gazebo logs (not constant speed), failure → return-to-charger cost |
| **Interruptible actions** | A CHARGE:100 cannot be stopped when a line blocks; a WAIT cannot be cut short | Re-ask the decision service during CHARGE/WAIT every N seconds with a "continue" option (option interruption / call-and-return with preemption) |
| **Behaviour trees for execution** | The ladder and phases are hand-written imperative code | `py_trees_ros` or BehaviorTree.CPP: `Sequence(Navigate, VerifyArrival, Dwell, Pickup)` inside `Retry`/`Fallback` nodes, visualised live with Groot2 |
| **Use AMCL, not ground truth, in `nearest_poi`** | Consistency with the "real robot" principle | `min(pois, key=dist(self.amcl_xy, poi))` |
| **Correct summary fields** | `seed` and `ended_by` can be misleading in autonomous runs | Pass the factory seed to the executor; `ended_by: shift_over` |
| **Warm-up request** | First-decision latency includes model loading | Call `/decide` once with a dummy state at executor startup, or load models at service startup |
| **Nav2 `docking_server` for CHARGE** | Real charging needs precise contact | `DockRobot` action with an AprilTag dock pose, `UndockRobot` before the next drive |
| **Hardware-independent safety** | Software e-stop is not a safety function | For a real robot: safety PLC + safety-rated lidar fields; ROS only requests stops, the safety chain enforces them |
