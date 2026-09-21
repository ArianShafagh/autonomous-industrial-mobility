# 14 — Live integration: decision service, LiveState, dashboard, launch and run scripts

Files:
- `src/robofetch_ai/robofetch_ai/service.py`: decision service (FastAPI, port 8001)
- `src/robofetch_ai/robofetch_ai/env/live_state.py`: `LiveState`
- `src/robofetch_core/robofetch_core/mission_executor.py`: `build_state`, `next_action`, `_fallback`, `_publish_decision`
- `src/robofetch_bridge/robofetch_bridge/app.py` + `ros_link.py`: dashboard (FastAPI, port 8000)
- `src/robofetch_web/web/templates/dashboard.html`, `static/style.css`
- `src/robofetch_bringup/launch/mission.launch.py`
- `scripts/run.sh`, `scripts/stop.sh`

This is where the offline AI becomes a robot that works a shift by itself.

---

## 1. Why a separate decision service

From the `service.py` docstring:
1. *"'the AI is unavailable' has to be a state you can produce by stopping a process, so the robot's fallback behaviour is testable rather than theoretical."*
2. *"The models here are the ones evaluated in the fast simulator — same code, same weights — so what the thesis measures offline is what drives the robot."*

Practical reasons as well: torch, SB3 and FastAPI live in the venv, while the ROS nodes run with ROS's Python. The service has **no ROS dependency** at all, so it could run on another machine or GPU box.

---

## 2. The decision service API

Started by the launch file as `venv/bin/python -m uvicorn robofetch_ai.service:app --host 0.0.0.0 --port 8001`.

| Method | Path | Returns |
|---|---|---|
| `GET` | `/health` | `{"ok": true, "loaded": ["ns/balanced"], "decisions": 17, "failures": 0, "models": [...]}` |
| `GET` | `/models` | `{"models": ["ns", "ns_symbolic_only", "ns_neural_only", "ppo", "rule"]}` |
| `POST` | `/decide` | see below |

### 2.1 Request

```json
{
  "model": "ns",
  "scenario": "balanced",
  "state": {
    "time_s": 145.8, "location": "A",
    "battery_percent": 98.9, "temperature_c": 25.1, "condition_percent": 100.0,
    "payload_kg": 1.3, "cargo_units": 3, "shift_duration_s": 900.0,
    "sections": {
      "A": {"status": "RUNNING", "status_code": 0, "produced_total": 4, "picked_total": 3,
            "buffer_units": 0, "buffer_capacity": 8, "buffer_fill": 0.0, "unit_mass_kg": 0.5,
            "buffer_mass_kg": 0.0, "rate_nominal_per_hour": 20.0, "rate_actual_per_hour": 20.0,
            "health_percent": 99.6, "time_to_full_s": 1402.0, "fault_remaining_s": 0.0,
            "faults_total": 0, "lost_units": 0.0, "blocked_time_s": 0.0, "factory_time_s": 146.0},
      "B": {"...": "..."}, "C": {"...": "..."}
    }
  }
}
```

`DecideRequest` is a **Pydantic** model: `state: dict`, `model: str = "ns"`, `scenario: str = ROBOFETCH_SCENARIO env or "balanced"`. FastAPI validates the JSON body against it.

### 2.2 Handling

```python
@app.post("/decide")
def decide(request: DecideRequest):
    live = context(request.scenario)                           # cached LiveState per scenario
    state = dict(request.state)
    for section in state["sections"].values():                 # JSON null -> Python inf / 0
        if section.get("time_to_full_s") is None:   section["time_to_full_s"] = math.inf
        if section.get("fault_remaining_s") is None: section["fault_remaining_s"] = 0.0
    live.update(**state)
    chosen = policy(request.model, request.scenario).decide(live.state(), live.legal_actions(), live)
    return {"action": "DELIVER", "explanation": "...", "scores": json_safe(scores),
            "model": "ns", "latency_ms": 5.21}
```

**Policy cache:** `_policies[(model, scenario)]` is built once. Loading a network on every request would cost more than deciding. The first request for a model therefore takes about 1.5 s (loading torch weights, and for PPO also importing SB3); later ones take 3–7 ms (HANDOVER WP7).

**Errors never crash the service:** any exception returns HTTP 200 with `{"error": "...", "traceback": "..."}` and increments `failures`. The executor treats an `"error"` key as a failure and falls back.

### 2.3 The infinity round trip

JSON has no `Infinity`. A section in FAULT has `time_to_full_s = inf`, and symbolic-only scores can be `-inf`. The chain:

| Step | Where | inf becomes |
|---|---|---|
| ROS msg `float32 time_to_full_s = inf` | factory_node | inf |
| `build_state()` | executor | `None` (`null` in JSON) |
| `/decide` handler | service | `math.inf` again |
| scores ±inf | `NeuroSymbolicPolicy._decision` | `None` |
| `json_safe()` | service | any remaining non-finite float → `None` |

HANDOVER WP7 problem 4: before this, symbolic-only decisions could not be sent at all.

---

## 3. `LiveState`: the real world through the simulator's interface

```python
class LiveState:
    def __init__(self, scenario="balanced", config_dir=None):
        self.cfg = load_config(scenario); self.p = RobotParams.from_config(self.cfg)
        self.matrix = load_path_matrix(); self.sim_cfg = cfg["mission"]["simulation"]
        self.objective = cfg["mission"]["objective"]; self.shift_duration_s = cfg["time"]["shift_duration_s"]

    def update(self, *, time_s, location, battery_percent, temperature_c, condition_percent,
               payload_kg, cargo_units, sections, shift_duration_s=None):
        # adds per section: distance_m (from location), units_that_fit (from free payload);
        # status from status_text if missing; builds the same dict FactorySim.state() returns
    def state(self): ...
    def charge_targets(self): ...
    def legal_actions(self): ...     # the same rule as FactorySim.legal_actions, on live numbers
    @property sections / robot       # what PPO's observation code reads
```

This is the **adapter pattern**: the policies were written against `FactorySim`, and `LiveState` makes the live system look identical (`state()`, `legal_actions()`, `p`, `matrix`, `shift_duration_s`, `charge_targets()`). The docstring says: *"the code that decides in the fast simulator is literally the same code that decides on the robot. No second implementation, no chance of the two drifting apart."*

**Scenario matters live.** `LiveState(scenario)` loads that scenario's robot parameters, so the symbolic rules on the robot use, for example, the aged battery's 12 Wh capacity.

**Concurrency note:** `_contexts[scenario]` is one shared `LiveState` whose `_state` is overwritten by `update()`. FastAPI runs `def` endpoints in a **thread pool**, so two simultaneous `/decide` calls for the same scenario could race (request A updates, request B updates, request A decides on B's state). With one robot asking sequentially this never happens. For several robots it would be a real bug. The fix is a new `LiveState` per request (it is cheap once the config is cached) or a lock.

---

## 4. The executor side: asking, falling back, reporting

`next_action()`:

```python
state = self.build_state()                         # from SectionStatus msgs + telemetry + own bookkeeping
if self.model == "fallback":                       # "no AI" on purpose
    action, why = self._fallback(state); publish(model="fallback", fallback=False); return
try:
    POST decision_url with {"model", "scenario", "state"}  (urllib, timeout 5 s)
    if "error" in answer: raise RuntimeError(answer["error"])
    action = parse_action(answer["action"])
    publish(model=self.model, fallback=False, scores, latency)
except (URLError, OSError, KeyError, ValueError, RuntimeError):
    decision_stats failed += 1, fallback += 1
    log "decision service unavailable (...); using the fallback rules"
    action, why = self._fallback(state); publish(model="fallback", fallback=True)
```

`/mission/decision` JSON: `{"seq", "sim_time_s", "action", "explanation", "model", "scores", "latency_ms", "fallback", "battery_percent", "location"}`.

**Real fallback test (HANDOVER WP7, `logs/wp7_fallback.log`):** the service was killed mid-shift. The model answered 2 decisions. The executor then logged `decision service unavailable (Connection refused); using the fallback rules` and finished the shift on the rules (5 decisions, e.g. `fallback: carrying 4 units, nothing to collect`). The robot never stopped. The summary said `answered_by_model: 2, fallback: 5`.

**Timeout reasoning:** 5 s covers the first model load (~1.5 s). A decision that takes longer is treated as a failure. The robot does not stand still waiting.

**Not handled:** if the service *answers* with an action that is illegal in the executor's view (e.g. a stale state), the executor executes it anyway. The PICKUP would simply return 0 units. The service computes legality from the same state, so this cannot normally happen.

---

## 5. The dashboard (read-only monitor)

### 5.1 Architecture

```
uvicorn process (port 8000)
├── FastAPI app (app.py)
│     GET /            → Jinja2 renders dashboard.html from snapshot()
│     GET /api/state   → snapshot() as JSON
│     GET /health      → connected? sections? decisions seen?
│     /static          → style.css   (follow_symlink=True for --symlink-install)
└── RosThread (ros_link.py) — started lazily on first request
      rclpy.init(); RosLink node (use_sim_time) on a SingleThreadedExecutor in a daemon thread
      subscribes: /factory/A|B|C/status, /robot/telemetry, /mission/decision, /mission/events, /amcl_pose
      stores latest values under a threading.Lock; deques of the last 400 decisions/events
```

The HTTP handlers run in other threads, so every read and write of shared data goes through `self.lock`. `snapshot()` returns plain Python data (dicts/lists) that are safe to serialise.

**There is no publisher, no button, no login.** From the docstring: *"The dashboard is a window, not a control panel."* The only intervention is `./scripts/stop.sh`.

### 5.2 The page

Rendered **server-side**, refreshed by `<meta http-equiv="refresh" content="2">`, with **no JavaScript** (the CSS comment says a 1 Hz system does not need a frontend build). It shows:
- **Section cards:** status tag (coloured by status), buffer bar, units waiting, produced, collected, actual/nominal rate, health, "full in", lost production, repair time left.
- **Robot card:** battery bar, activity, payload, motor temperature, condition, AMCL position, distance, energy.
- **Shift card:** units delivered, produced, lost, energy per unit.
- **Decision table:** time, model (marked fallback), action, reason, battery, latency. Fallback rows are highlighted.
- **Actions table:** seq, action, result detail, time, distance and energy as *measured / predicted*, units, battery. Failed rows are highlighted.
- Light/dark theme via `prefers-color-scheme`.

### 5.3 Details and limitations

- **Delivered total** is counted from `/mission/events` DELIVER SUCCEEDED messages **seen by the dashboard**. If the dashboard starts after deliveries have happened, the count is too low. (The executor's summary is authoritative.) The per-section `delivered` dict is initialised but only `_total` is used.
- **Energy per unit** = cumulative robot energy / delivered, which includes energy spent before the first delivery.
- `/api/state` includes `energy_history` (last 120 telemetry samples: battery, energy, temperature), which is **not drawn** on the page. It is meant for thesis figures.
- HANDOVER WP7 problem 3: Starlette's `TemplateResponse(request, name, context)` signature. Passing the name first gave "unhashable type: dict".

---

## 6. Launch orchestration (`mission.launch.py`)

### 6.1 Arguments

| Argument | Default | Effect |
|---|---|---|
| `scenario` | balanced | factory node, robot model, executor, and the service request |
| `seed` | −1 | factory seed (−1 = scenario's `time.seed`) |
| `model` | **ns** | decision model; `""` = scripted `plan`; `fallback` = no AI |
| `plan` | "" | scripted actions (used when `model` is empty) |
| `ai` | true | start the decision service |
| `web` | true | start the dashboard |
| `shift_s` | 0.0 | shift length (0 = from `params.yaml`, 3600 s) |
| `headless` | false | Gazebo server only, no RViz |
| `planner` | NavfnDijkstra | global planner (WP8, uncommitted) |
| `rviz` | true | RViz when not headless |

### 6.2 Start order

```
t = 0 s   navigation(headless) OR navigation(GUI)   ← two IncludeLaunchDescription, exactly one runs (If/UnlessCondition)
t = 3 s   decision_service (if ai) , dashboard (if web)          ← TimerAction
t = 5 s   factory (scoped include), robot_state_node
t = 8 s   mission_executor
```

The timers are coarse. Correctness does not depend on them, because the executor explicitly waits for Nav2 active, telemetry, pickup services and AMCL. The timers only spread CPU load during Gazebo/Nav2 start-up.

All nodes get `use_sim_time: true`, and all share one `run_id = time.strftime("run_%Y%m%d_%H%M%S")` computed when the launch file is loaded, so all log files of a run line up.

`workspace_python()` resolves the venv interpreter with `os.path.realpath` (the symlink-install lesson, file 02 §10).

---

## 7. `run.sh`: the user entry point

### 7.1 Interactive mode (no arguments)

1. Lists the 12 scenarios with their `description:` lines and the 5 decision modes (`ns`, `ns_symbolic_only`, `ppo`, `rule`, `fallback`).
2. Asks for scenario number (default balanced), model number (default 1 = ns), view (1 GUI / 2 headless) and shift minutes (default 10). Invalid input is asked again.
3. Shows a summary and asks `Start? (y/n)`.
4. Builds launch arguments; `fallback` adds `ai:=false`.

`ns_neural_only` is available in the service but deliberately **not offered** in the menu (it is unsafe by design).

### 7.2 Non-interactive mode (any argument)

```bash
./scripts/run.sh scenario:=heavy_load model:=ppo shift_s:=900.0 --headless --no-build
./scripts/run.sh model:=fallback                        # no AI
./scripts/run.sh model:= plan:="PICKUP:B;DELIVER"      # scripted
./scripts/run.sh --list
```

`model:=` without a plan uses the demo plan `PICKUP:B;PICKUP:A;DELIVER;PICKUP:C;DELIVER;CHARGE:100`.

### 7.3 Then

It refuses to start if `stop.sh --running` finds simulation processes (two `/clock` publishers break both runs). Then it sources ROS, runs `colcon build --symlink-install` (unless `--no-build`), sources `install/setup.bash`, and `exec ros2 launch robofetch_bringup mission.launch.py …`.

---

## 8. `stop.sh`: the only way to stop

- **`PATTERN`** matches process names (`ps -o comm`, truncated to 15 chars, hence prefixes like `mission_executo`, `robot_state` covering both `robot_state_publisher` and `robot_state_node`, and `route_server`).
- **`API_PATTERN`** matches full command lines for the two uvicorn servers (their process name is just `python`).
- Signals: TERM, TERM, KILL with 3 s pauses, then it reports anything still alive.
- **`--running`** prints the count and exits 0 if any process is alive (used by run scripts to refuse to start).
- **`--check`** lists live ROS-looking processes the pattern would *miss*.

The long comments record costly real incidents: orphaned `robot_state_node`s all publishing telemetry, and a surviving `route_server` breaking every subsequent Nav2 bringup. HANDOVER also warns: never `pkill -f` with a pattern that appears in your own shell's command line (it killed the calling shell twice); kill by port instead (`fuser -k 8001/tcp`).

---

## 9. Security note

Both services bind to **0.0.0.0** with no authentication:
- anyone on the network can read the full factory and robot state on port 8000;
- anyone can `POST /decide`. That does not command the robot (the executor only *asks*), but it can load models and use CPU.

For a lab this is fine. For anything beyond it, bind to `127.0.0.1` or put the services behind a reverse proxy with authentication.

---

## 10. Key concepts of this section

- Microservice boundary for the AI; failure injection by killing a process
- REST API design with FastAPI + Pydantic validation
- Model caching and cold-start latency
- Adapter pattern (`LiveState` ↔ `FactorySim`)
- Graceful degradation (fallback), observability of degraded mode
- JSON limitations (no ±∞/NaN) and sentinel conversion
- Embedding an rclpy node in a web server thread; lock-protected shared state
- Server-side rendering without JavaScript
- Launch orchestration: conditional includes, timers, argument forwarding
- Process lifecycle hygiene in ROS systems

---

## 11. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Per-request `LiveState` or a lock** | Race on concurrent requests (multi-robot) | Build `LiveState` in the handler from a cached config, or protect `update+decide` with `threading.Lock` |
| **Warm-up at service start** | First-decision latency ~1.5 s | FastAPI `lifespan` handler that constructs `ns`/`ppo` policies for the launch scenario |
| **Typed API schema** | `state: dict` accepts anything; errors surface deep inside policies | Pydantic models for `RobotState` and `SectionState`; FastAPI then returns 422 with the exact missing field |
| **gRPC / ROS 2 service instead of HTTP+JSON** | JSON float/inf quirks, no schema | gRPC with protobuf (typed, fast, streaming), or a ROS 2 service in the venv with `rclpy` |
| **Model serving formats** | torch + SB3 import cost; portability to embedded hardware | Export the scorer and PPO actor to **ONNX** (`torch.onnx.export`), run with `onnxruntime` (ms-level, tiny dependency); **TorchScript** as an alternative |
| **Live map on the dashboard** | HANDOVER open issue: robot position is shown as numbers | Draw `factory_maze.pgm` + POIs + robot pose as inline SVG; or embed **Foxglove Studio** (connects to ROS 2 via `foxglove_bridge`, with plots, maps and logs) |
| **Push updates instead of refresh** | Full page reload every 2 s | Server-Sent Events or WebSockets with small JS; or keep the no-JS design and use `htmx` partial refreshes |
| **Metrics and alerting** | Fallback rate, latency, battery | Prometheus endpoint (`prometheus-fastapi-instrumentator`) + Grafana; alert if fallback rate > 0 or battery < reserve |
| **Bind to localhost / auth** | Open ports | `--host 127.0.0.1`; or reverse proxy with basic auth / OAuth |
| **Health-based fallback before timeout** | A hung service costs 5 s per decision | Executor checks `/health` in the background; if unhealthy, use fallback immediately and retry the service every 30 s (circuit breaker pattern) |
