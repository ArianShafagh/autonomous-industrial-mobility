# 02 — ROS 2 foundations, as used in this project

You can understand the AI without ROS. But to see how a decision becomes a moving robot, and why several design choices were made (sim time, one factory process, lifecycle waits), you need these concepts. Each is explained through the place it appears in this workspace.

ROS 2 distribution: **Jazzy Jalisco**. Simulator: **Gazebo Harmonic (gz-sim 8)**. Build tool: **colcon**. Python client library: **rclpy**.

---

## 1. Nodes

A **node** is one participant in the ROS graph: a process (or part of one) that publishes, subscribes, offers services or actions. In Python:

```python
class FactoryNode(Node):
    def __init__(self):
        super().__init__("factory")          # node name on the network
```

Nodes in this project:

| Node name | Code | Role |
|---|---|---|
| `factory` | `robofetch_factory/factory_node.py` | Steps the 3 production sections, publishes their status, serves pickups |
| `robot_state_node` | `robofetch_core/robot_state_node.py` | Integrates the battery/heat/wear model from odometry |
| `mission_executor` | `robofetch_core/mission_executor.py` | Decides (via service) and executes actions |
| `robofetch_monitor` | `robofetch_bridge/ros_link.py` | Dashboard's listener, runs inside the web process |
| `factory_monitor` | `robofetch_factory/monitor.py` | Terminal table |
| `planner_bench`, `check_nav` | `tools/nav/compare_planners.py`, `scripts/check_nav.py` | Test tools |
| Nav2 nodes | from `nav2_bringup` | `map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `smoother_server`, `velocity_smoother`, `collision_monitor`, `waypoint_follower`, `route_server`, `docking_server`, two `lifecycle_manager`s |
| `robot_state_publisher` | ROS package | Publishes the URDF link transforms |
| `parameter_bridge` | `ros_gz_bridge` | Converts Gazebo topics ↔ ROS topics |

---

## 2. Topics: publish/subscribe

A **topic** is a named, typed, many-to-many data stream. Publishers do not know who listens. Example from `factory_node.py`:

```python
self.pubs[sid] = self.create_publisher(SectionStatus, f"/factory/{sid}/status", 10)
...
self.pubs[sid].publish(msg)
```

and the subscriber side in `mission_executor.py`:

```python
self.create_subscription(SectionStatus, f"/factory/{sid}/status",
                         lambda msg, sid=sid: self.sections.__setitem__(sid, msg), 10, ...)
```

The `10` is the **QoS history depth** (keep the last 10 messages if the subscriber is slow). One detail: the lambda uses `sid=sid` as a default argument. Without it, every lambda would capture the *last* value of the loop variable (Python late binding), and all three subscriptions would write into section C.

**QoS (Quality of Service)** also covers reliability and durability. `check_nav.py` publishes `/initialpose` with `DurabilityPolicy.TRANSIENT_LOCAL`, which means late-joining subscribers (AMCL) still receive the last message. The same idea is used by `map_server` for `/map` (`map_subscribe_transient_local: True` in `nav2_params.yaml`).

---

## 3. Messages and interfaces

Messages are defined in `.msg` files and compiled into Python/C++ classes by `rosidl` when `robofetch_interfaces` (an `ament_cmake` package) is built.

### `SectionStatus.msg` (published by the factory)

Key design points in the comments:
- Constants `RUNNING=0, DEGRADED=1, BLOCKED=2, FAULT=3` are an enum in a message.
- **Durations are simulation seconds, rates are per factory hour.** `factory seconds = simulation seconds × time_scale`.
- `time_to_full_s` can be `inf` (while FAULT). JSON cannot encode infinity, which is why the executor converts it to `null` and the service back to `math.inf` (file 14).

### `TaskEvent.msg` (published by the executor)

One message per action phase (`STARTED`, `SUCCEEDED`, `FAILED`), carrying **measured and predicted** duration, distance and energy side by side. The dashboard and the thesis figures use this.

### `Pickup.srv` (service)

```
int32 max_units -1           # -1 = no unit limit
float32 max_mass_kg -1.0     # <= 0 = no mass limit
---
bool success
uint32 units
float32 mass_kg
uint32 buffer_left
string message
```

The part above `---` is the request and the part below is the response. Default values (`-1`) make optional fields possible.

---

## 4. Services: request/response

A **service** is a synchronous call with one server and many clients. It is used for *short* operations. Server in `factory_node.py`:

```python
self.create_service(Pickup, f"/factory/{sid}/pickup",
                    lambda req, res, sid=sid: self._on_pickup(sid, req, res))
```

Client in `mission_executor.py`:

```python
req = Pickup.Request()
req.max_units = -1
req.max_mass_kg = float(max(0.0, self.p.max_payload_kg - self.payload_kg))
res = self._wait(client.call_async(req), 10.0)
```

`call_async` returns a **future**. The executor's worker thread blocks on it with `_wait`, which uses a `threading.Event` set by a done-callback, while the ROS executor thread delivers the response. This pattern (worker thread + spinning executor) avoids the classic rclpy deadlock of calling `spin_until_future_complete` inside a callback.

Other services used: `std_srvs/Trigger` on `/lifecycle_manager_navigation/is_active`, and `nav2_msgs/ClearEntireCostmap` for recovery.

---

## 5. Actions: long-running goals with feedback and cancel

An **action** is for tasks that take time: send goal → accepted/rejected → feedback stream → result. It can be cancelled. Nav2 exposes driving as the `NavigateToPose` action.

In `mission_executor._drive_once`:

```python
handle = self._wait(self.nav_client.send_goal_async(goal), 30.0)      # 1. goal accepted?
if handle is None or not handle.accepted:
    return False, "goal rejected by Nav2"
result = self._wait(handle.get_result_async(), timeout_s)               # 2. wait for result
if result is None:                                                      # 3. timed out
    self._wait(handle.cancel_goal_async(), 10.0)                        #    -> cancel
...
if result.status == 4:          # action_msgs/GoalStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
```

Also used: `BackUp` (recovery), and in the WP8 tool `ComputePathToPose` (plan only) and `FollowPath` (control only). That split lets the planner comparison isolate the planner from the controller.

---

## 6. TF: coordinate frames

**TF2** keeps a time-stamped tree of coordinate frames. This robot's tree:

```
map ──(AMCL)──► odom ──(Gazebo DiffDrive plugin)──► base_footprint ──(URDF fixed)──► base_link
                                                                                     ├── left_wheel  (continuous joint, from /joint_states)
                                                                                     ├── right_wheel
                                                                                     ├── front_caster, rear_caster
                                                                                     └── lidar_link
```

- **`map → odom`** is published by AMCL. It corrects for accumulated odometry drift, so `map` is globally consistent but can jump.
- **`odom → base_footprint`** is published by wheel odometry. It is smooth but drifts over time.
- **`base_footprint`** is a frame on the ground under the robot centre (REP-105 convention). `base_link` is lifted by the wheel radius, 0.06 m.

**Important project fact:** the generator (file 03) makes the occupancy map's origin match the Gazebo world frame, so **map coordinates == world coordinates**. A POI `(x=6.025, y=3.075)` is the same point in Gazebo, in RViz and for Nav2.

The WP3 bug "*transform from base_footprint to map did not become available*" is a TF problem. The planner cannot activate until `map→odom` exists, and AMCL publishes it only after receiving an initial pose. The fix is in file 04.

---

## 7. Simulation time (`use_sim_time`)

Gazebo publishes `/clock`. Every node with `use_sim_time: true` uses that clock instead of the wall clock. Consequences in this project:

1. **All measured durations are sim seconds.** `mission_executor.sim_now()` reads `self.get_clock().now()`. `sim_sleep()` loops until the *sim* clock has advanced (Gazebo may run faster or slower than real time; headless it ran ~2× faster, per HANDOVER WP3).
2. **The factory and battery follow Gazebo.** If you pause Gazebo, production pauses too. WP2 revision fixed a bug where `robot_state_node` integrated on wall-clock time.
3. **CPU cost of `/clock`.** Each Python node on sim time processes every `/clock` message. At a 1 ms physics step (1000 Hz), the three section nodes plus the robot model used about 55 % CPU *each*. Nav2's lifecycle service replies got lost, and bringup hung. The fixes were a **4 ms physics step (250 Hz)** in `layout.yaml → physics_step_s` and **one factory process** instead of three nodes (`factory_node.py` docstring explains this). The executor uses a `SingleThreadedExecutor` because a multi-threaded one used 76 % CPU mostly handling `/clock`.

---

## 8. Parameters

Nodes declare parameters with defaults, and launch files override them:

```python
self.declare_parameter("scenario", "balanced")
self.declare_parameter("shift_duration_s", 0.0)    # 0 = value from params.yaml
```

**Type strictness:** passing `shift_s:=900` (an integer) to a parameter declared as a double was rejected (HANDOVER WP7). The launch file fixes this with `ParameterValue(LaunchConfiguration("shift_s"), value_type=float)`.

The project keeps *domain* numbers out of ROS parameters. ROS parameters select *which* scenario and run ID. The numbers themselves come from `params.yaml` via `load_config()`, so the fast simulator (which has no ROS) reads the same values.

---

## 9. Lifecycle (managed) nodes

Nav2 servers are **lifecycle nodes** with states `unconfigured → inactive → active → finalized`. A `lifecycle_manager` drives the transitions (`autostart: true`). Two lessons from this project:

- **"Action server exists" ≠ "navigation ready."** `bt_navigator` creates the `navigate_to_pose` server when *configured*, long before it is *active*. `wait_for_navigation_active()` in the executor polls `/lifecycle_manager_navigation/is_active` instead (docstring explains it).
- **One failing node aborts the whole bringup.** `route_server` (new in Jazzy) fails to configure without a graph file, so `nav2_params.yaml` points it at a stock sample graph, and `stop.sh` must also kill it. Otherwise a leftover `route_server` from a previous run answers lifecycle calls and breaks the next bringup (long comment in `stop.sh`).

---

## 10. Launch files

Python launch files describe what to start. Concepts used in `mission.launch.py` / `navigation.launch.py`:

| Construct | Meaning | Example here |
|---|---|---|
| `DeclareLaunchArgument` | a `name:=value` CLI argument with default | `scenario`, `model`, `ai`, `web`, `shift_s`, `headless`, `planner`, `rviz`, `seed`, `plan` |
| `LaunchConfiguration("x")` | a lazy reference to an argument's value | passed into node parameters |
| `IncludeLaunchDescription` | run another launch file | navigation, factory |
| `GroupAction(scoped=True, forwarding=True)` | scope argument changes to the group, but still see parent args | fixed "launch configuration 'scenario' does not exist" (WP3) and `rviz:=false` leaking into the parent (comment in `navigation.launch.py`) |
| `IfCondition` / `UnlessCondition` | start only if true/false | headless vs GUI include; `ai:=false` skips the service |
| `TimerAction(period=…)` | delay a start | service/dashboard at 3 s, factory/robot model at 5 s, executor at 8 s |
| `ExecuteProcess` | run a non-ROS command | the two uvicorn servers with the venv Python |
| `OpaqueFunction` | run Python at launch time with resolved arguments | writes a temporary Nav2 params file with AMCL's initial pose and the chosen planner |
| `SetEnvironmentVariable` | env var for children | `QT_QPA_PLATFORM=xcb` so Gazebo's window appears on WSLg |

**`realpath` vs `abspath` (WP7 bug):** with `colcon build --symlink-install`, `install/.../mission.launch.py` is a symlink into `src/`. `workspace_python()` must resolve the symlink (`os.path.realpath`) to find `<ws>/venv/bin/python`. With `abspath` it looked for a venv inside `install/`, fell back to system `python3` (no FastAPI), and the decision service died at launch.

---

## 11. Building: colcon, ament, symlink install

- **`ament_cmake`** packages (interfaces, description, gazebo, nav, web, bringup) install data files with CMake `install(DIRECTORY ...)`.
- **`ament_python`** packages (factory, core, ai, bridge) use `setup.py`. `console_scripts` entry points become executables for `ros2 run`. For example, `mission_executor = robofetch_core.mission_executor:main`.
- **`--symlink-install`** links Python files and data into `install/` instead of copying them, so an edit in `src/` takes effect without rebuilding. Stale files can remain in `install/` or `build/` after renames or deletions, and several HANDOVER problems were exactly that (`warehouse.*`, `factory.yaml`, the old `monitor` executable).
- **`COLCON_IGNORE`** in `venv/` and `tools/` stops colcon from finding stray `setup.py` files.

### The venv + ROS combination

```bash
python3 -m venv --system-site-packages venv    # so rclpy (from /opt/ros/jazzy) is importable
venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
```

The pins come from real breakages. **numpy < 2** is needed because ROS Jazzy's Python packages are compiled against numpy 1.x ABI. **setuptools < 80** is needed because colcon-core 0.21 requires it and ament_python builds break otherwise.

---

## 12. Key concepts of this section

- **Publish/subscribe decoupling:** the dashboard could be added without touching the executor. It just subscribes.
- **Synchronous (service) vs asynchronous long-running (action) interfaces:** choose by duration and whether you need cancel/feedback.
- **Futures + worker thread** to avoid blocking the executor that must deliver the very response you are waiting for.
- **Sim time** makes measurements independent of how fast the simulator runs, at a CPU cost per node.
- **Lifecycle management** gives deterministic startup, but one failing node blocks all of them.

---

## 13. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Typed messages instead of JSON strings** | Schema checking, `ros2 topic echo` readability, bag introspection | New `.msg` files (see file 01) |
| **Composable nodes / C++ for sim-time heavy nodes** | Python `/clock` handling cost ~30 % CPU per node | Put factory + robot model in one `rclpy` process with a shared executor, or rewrite as C++ components loaded into one container |
| **`ros2_control` + `diff_drive_controller`** | The Gazebo DiffDrive plugin is simulation-only. `ros2_control` is the same interface a real robot uses | Add `<ros2_control>` tags + `gz_ros2_control`, spawn `diff_drive_controller` and `joint_state_broadcaster` |
| **`launch_testing`** | Nothing tests that the launch actually brings the system up | A launch test that starts `mission.launch.py headless:=true shift_s:=120` and asserts a summary YAML with `ended_by: plan_finished` |
| **Zenoh RMW (`rmw_zenoh`)** | Default DDS discovery is chatty and flaky on WSL. Zenoh is the Jazzy-era alternative | `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` and run `rmw_zenohd` |
| **Real-time factor control** | Headless Gazebo ran ~2× real time. Faster-than-real-time sim would speed up Gazebo experiments | `<real_time_factor>0</real_time_factor>` (as fast as possible) in the generated SDF for batch runs; everything already uses sim time |
