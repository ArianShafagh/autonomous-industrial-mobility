"""Mission executor — carries out robot actions in the factory, with no human involved.

    PICKUP <section> | DELIVER | CHARGE <percent> | WAIT <seconds>     (see mission_plan.py)

Where the next action comes from is pluggable. For now (WP3) it is a scripted `plan` parameter;
the AI decision service replaces it later without changing how actions are executed.

    uses      Nav2 NavigateToPose                   driving
              /factory/<id>/pickup                  taking units from a section
    reads     /robot/telemetry                      battery, temperature, condition (robot_state_node)
              /model/robofetch/pose                 ground-truth pose: measured distance
              /amcl_pose                            localisation ready
    publishes /robot/activity                       what the robot does and carries (for the model)
              /mission/events  (TaskEvent)          every action, predicted vs measured
    writes    logs/<run_id>_mission.csv             one row per finished action
              logs/<run_id>_mission_summary.yaml    totals for the run

Reused from the old task manager: the Nav2 goal handling with retries, the AMCL initial-pose
handshake, and the emergency stop (script-only, /robot/estop) that cancels the goal and brakes.
The worker thread blocks on futures while a MultiThreadedExecutor serves the callbacks.
"""
import csv
import json
import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from builtin_interfaces.msg import Duration
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robofetch_core.mission_plan import (CHARGE, CHARGER, DELIVER, DELIVERY, PICKUP, WAIT,
                                         parse_plan, predict)
from robofetch_core.paths import default_log_dir
from robofetch_core.robot_model import RobotParams
from robofetch_factory.factory_model import load_config
from robofetch_factory.layout import load_path_matrix, load_pois
from robofetch_interfaces.msg import TaskEvent
from robofetch_interfaces.srv import Pickup

CSV_FIELDS = ["seq", "action", "target", "phase", "detail", "sim_start_s", "duration_s",
              "predicted_duration_s", "distance_m", "predicted_distance_m", "energy_wh",
              "predicted_energy_wh", "charged_wh", "predicted_charged_wh",
              "battery_start_percent", "battery_end_percent", "predicted_battery_end_percent",
              "payload_kg", "units", "location", "recovery"]

# action_msgs/GoalStatus
STATUS_TEXT = {0: "unknown", 1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded",
               5: "canceled", 6: "aborted"}


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionExecutor(Node):
    def __init__(self):
        super().__init__("mission_executor")
        self.declare_parameter("scenario", "balanced")
        self.declare_parameter("plan", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("log_dir", default_log_dir())
        self.declare_parameter("charge_check_period_s", 2.0)

        cfg = load_config(self.get_parameter("scenario").value)
        self.cfg = cfg
        self.p = RobotParams.from_config(cfg)
        self.nav_cfg = cfg["mission"]["navigation"]
        self.pois = load_pois()
        self.matrix = load_path_matrix()
        self.plan = parse_plan(self.get_parameter("plan").value)
        self.run_id = self.get_parameter("run_id").value or time.strftime("run_%Y%m%d_%H%M%S")

        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                       callback_group=group)
        self.pickup_clients = {sid: self.create_client(Pickup, f"/factory/{sid}/pickup",
                                                       callback_group=group)
                               for sid in ("A", "B", "C")}
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.activity_pub = self.create_publisher(String, "/robot/activity", 10)
        self.event_pub = self.create_publisher(TaskEvent, "/mission/events", 10)
        self.halt_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_active_client = self.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active", callback_group=group)
        self.backup_client = ActionClient(self, BackUp, "backup", callback_group=group)
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, f"/{m}_costmap/clear_entirely_{m}_costmap",
                               callback_group=group) for m in ("global", "local")]

        self._amcl_ready = threading.Event()
        self.telemetry = None
        self._truth = None
        self.distance_travelled = 0.0          # ground truth, whole run
        self.amcl_xy = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10,
                                 callback_group=group)
        self.create_subscription(String, "/robot/telemetry", self._on_telemetry, 10,
                                 callback_group=group)
        self.create_subscription(Pose, "/model/robofetch/pose", self._on_truth, 10,
                                 callback_group=group)
        self.create_subscription(String, "/robot/estop", self._on_estop, 10, callback_group=group)

        self._abort = threading.Event()
        self._nav_handle = None
        self._nav_lock = threading.Lock()

        # Robot's own bookkeeping
        self.location = CHARGER                  # the robot spawns on the charger
        self.payload_kg = 0.0
        self.cargo = {}                          # section -> units on board
        self.delivered = {"A": 0, "B": 0, "C": 0}
        self.seq = 0
        self.rows = []
        self.stuck = False                       # charger unreachable after recovery: needs a human
        self.nav_stats = {"drives": 0, "drive_attempt_failures": 0, "timeouts": 0,
                          "recovered_after_retry": 0, "goals_abandoned": 0,
                          "returned_to_charger": 0}

        log_dir = self.get_parameter("log_dir").value
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f"{self.run_id}_mission.csv")
        self.summary_path = os.path.join(log_dir, f"{self.run_id}_mission_summary.yaml")
        self._csv_fh = open(self.csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_fh, fieldnames=CSV_FIELDS)
        self._csv.writeheader()

        self.get_logger().info(f"mission {self.run_id}: {len(self.plan)} scripted actions, "
                               f"scenario {cfg['name']}, logging {self.csv_path}")
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    # ------------------------------------------------------------------ inputs
    def _on_amcl(self, msg):
        self.amcl_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._amcl_ready.set()

    def _on_telemetry(self, msg):
        try:
            self.telemetry = json.loads(msg.data)
        except ValueError:
            pass

    def _on_truth(self, msg):
        xy = (msg.position.x, msg.position.y)
        if self._truth is not None:
            self.distance_travelled += math.dist(xy, self._truth)
        self._truth = xy

    def _on_estop(self, _msg):
        if self._abort.is_set():
            return
        self._abort.set()
        self.get_logger().error("EMERGENCY STOP - cancelling the mission and halting.")
        with self._nav_lock:
            handle = self._nav_handle
        if handle is not None:
            handle.cancel_goal_async()
        for _ in range(10):
            self.halt_pub.publish(Twist())
            time.sleep(0.1)
        self._activity("stopped")

    # ------------------------------------------------------------------ helpers
    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def sim_sleep(self, seconds):
        """Sleep in SIMULATION time (Gazebo may run slower or faster than the wall clock)."""
        end = self.sim_now() + seconds
        while self.sim_now() < end and not self._abort.is_set():
            time.sleep(0.05)

    def _wait(self, future, timeout=300.0):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return future.result() if done.wait(timeout) else None

    def battery(self):
        return float(self.telemetry["battery_percent"]) if self.telemetry else 0.0

    def energy_counters(self):
        t = self.telemetry or {}
        return float(t.get("cumulative_energy_wh", 0.0)), float(t.get("cumulative_charged_wh", 0.0))

    def _activity(self, activity):
        self.activity_pub.publish(String(data=json.dumps(
            {"activity": activity, "payload_kg": self.payload_kg})))

    # ------------------------------------------------------------------ navigation
    def publish_initial_pose(self, timeout=120.0):
        spawn = self.pois[CHARGER]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = spawn["x"], spawn["y"]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = yaw_to_quat(spawn["yaw"])
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            if self._amcl_ready.wait(2.0):
                return True
        return False

    def wait_for_navigation_active(self, timeout=240.0):
        """Block until Nav2's navigation lifecycle manager reports every node ACTIVE.

        The action server exists as soon as bt_navigator is configured, long before it accepts
        goals, so "server available" is not "navigation ready". If bringup never completes (a
        lost lifecycle response under load, WP3) this says so instead of hanging silently.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.nav_active_client.wait_for_service(timeout_sec=2.0):
                res = self._wait(self.nav_active_client.call_async(Trigger.Request()), 5.0)
                if res is not None and res.success:
                    return True
            time.sleep(2.0)
        return False

    def _drive_once(self, poi_name, timeout_s):
        """One Nav2 goal. Returns (succeeded, reason)."""
        poi = self.pois[poi_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = poi["x"], poi["y"]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(poi["yaw"])
        handle = self._wait(self.nav_client.send_goal_async(goal), 30.0)
        if handle is None or not handle.accepted:
            return False, "goal rejected by Nav2"
        with self._nav_lock:
            self._nav_handle = handle
        try:
            result = self._wait(handle.get_result_async(), timeout_s)
            if result is None:
                # Still driving after the time limit: stuck or oscillating. Stop it.
                self._wait(handle.cancel_goal_async(), 10.0)
                self.nav_stats["timeouts"] += 1
                return False, f"timeout after {timeout_s:.0f} s"
        finally:
            with self._nav_lock:
                self._nav_handle = None
        if self._abort.is_set():
            return False, "emergency stop"
        if result.status == 4:
            # Verify with the robot's OWN localisation (not Gazebo ground truth - a real robot has
            # none): Nav2 may have "arrived" next to a blocked goal.
            time.sleep(0.5)
            if self.amcl_xy is not None:
                error = math.dist(self.amcl_xy, (poi["x"], poi["y"]))
                if error > float(self.nav_cfg["arrival_tolerance_m"]):
                    return False, (f"Nav2 reported success but the robot is {error:.2f} m from "
                                   f"{poi_name} (goal blocked?)")
            return True, "succeeded"
        return False, f"Nav2 {STATUS_TEXT.get(result.status, result.status)}"

    def _clear_costmaps(self):
        for client in self.clear_clients:
            if client.wait_for_service(timeout_sec=2.0):
                self._wait(client.call_async(ClearEntireCostmap.Request()), 5.0)

    def _back_up(self):
        if not self.backup_client.wait_for_server(timeout_sec=2.0):
            return "backup behaviour unavailable"
        goal = BackUp.Goal()
        goal.target.x = float(self.nav_cfg["backup_distance_m"])
        goal.speed = float(self.nav_cfg["backup_speed_m_s"])
        goal.time_allowance = Duration(sec=int(goal.target.x / goal.speed * 3 + 5))
        handle = self._wait(self.backup_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return "backup rejected"
        result = self._wait(handle.get_result_async(), 60.0)
        return "backed up" if result is not None and result.status == 4 else "backup failed"

    def navigate_to(self, poi_name):
        """Drive to a POI with the recovery ladder. Returns (reached, recovery_steps).

        attempt 1 -> clear costmaps, attempt 2 -> back up, attempt 3 -> give up.
        Every step is returned so the event log shows exactly what happened.
        """
        distance = self.matrix[self.location][poi_name] if self.location in self.matrix else 20.0
        timeout = max(float(self.nav_cfg["timeout_min_s"]),
                      float(self.nav_cfg["timeout_factor"]) * distance / self.p.speed_m_s)
        steps = []
        self.nav_stats["drives"] += 1
        recoveries = [("clear costmaps", self._clear_costmaps), ("back up", self._back_up), None]
        for attempt, recovery in enumerate(recoveries, start=1):
            if self._abort.is_set():
                steps.append("emergency stop")
                break
            ok, reason = self._drive_once(poi_name, timeout)
            if ok:
                self.location = poi_name
                if attempt > 1:
                    self.nav_stats["recovered_after_retry"] += 1
                    self.get_logger().info(f"reached {poi_name} after recovery: {steps}")
                return True, steps
            self.nav_stats["drive_attempt_failures"] += 1
            steps.append(f"attempt {attempt}: {reason}")
            self.get_logger().warn(f"drive to {poi_name} failed ({reason}), attempt {attempt}")
            if reason == "emergency stop" or recovery is None:
                break
            name, fn = recovery
            outcome = fn()
            steps.append(f"recovery: {name}" + (f" ({outcome})" if outcome else ""))
            self.location = self.nearest_poi()
        self.location = self.nearest_poi()
        return False, steps

    def nearest_poi(self):
        if self._truth is None:
            return self.location
        return min(self.pois, key=lambda n: math.dist(self._truth,
                                                       (self.pois[n]["x"], self.pois[n]["y"])))

    # ------------------------------------------------------------------ actions
    def run_action(self, action):
        self.seq += 1
        start_loc = self.location
        pred = predict(self.p, self.matrix, action, start_loc, self.battery(), self.payload_kg,
                       float((self.telemetry or {}).get("temperature_c", self.p.ambient_c)),
                       float((self.telemetry or {}).get("condition_percent", 100.0)))
        t0, dist0 = self.sim_now(), self.distance_travelled
        e0, c0 = self.energy_counters()
        b0 = self.battery()
        self._event(action, "STARTED", "", t0, 0.0, pred, 0.0, 0.0, 0.0, b0, b0, 0)
        self.get_logger().info(
            f"[{self.seq}] {action} from {start_loc}: predicted {pred.distance_m:.1f} m, "
            f"{pred.duration_s:.0f} s, {pred.energy_wh:.2f} Wh")

        ok, detail, units = self._execute(action)

        time.sleep(1.2)            # let the next 1 Hz telemetry sample include the action's end
        e1, c1 = self.energy_counters()
        duration = self.sim_now() - t0
        row = self._event(action, "SUCCEEDED" if ok else "FAILED", detail, t0, duration, pred,
                          self.distance_travelled - dist0, e1 - e0, c1 - c0, b0, self.battery(),
                          units)
        self.get_logger().info(
            f"[{self.seq}] {action} {'done' if ok else 'FAILED'}: {detail} | measured "
            f"{row['distance_m']:.1f} m, {duration:.0f} s, {row['energy_wh']:.2f} Wh, "
            f"battery {b0:.1f} -> {row['battery_end_percent']:.1f} %")
        return ok

    def _execute(self, action):
        self.recovery_log = []
        if action.kind in (PICKUP, DELIVER, CHARGE):
            self._activity("driving")
            reached, steps = self.navigate_to(action.destination())
            self.recovery_log = steps
            if not reached:
                self._activity("waiting")
                return False, self._handle_unreachable(action.destination()), 0

        if action.kind == PICKUP:
            self._activity("loading")
            self.sim_sleep(self.p.load_time_s)
            client = self.pickup_clients[action.target]
            if not client.wait_for_service(timeout_sec=5.0):
                self._activity("waiting")
                return False, f"section {action.target} pickup service unavailable", 0
            req = Pickup.Request()
            req.max_units = -1
            req.max_mass_kg = float(max(0.0, self.p.max_payload_kg - self.payload_kg))
            res = self._wait(client.call_async(req), 10.0)
            if res is None:
                self._activity("waiting")
                return False, "pickup service did not answer", 0
            self.payload_kg += res.mass_kg
            self.cargo[action.target] = self.cargo.get(action.target, 0) + res.units
            self._activity("waiting")
            return True, res.message, res.units

        if action.kind == DELIVER:
            self._activity("unloading")
            self.sim_sleep(self.p.unload_time_s)
            units = sum(self.cargo.values())
            for sid, n in self.cargo.items():
                self.delivered[sid] += n
            self.cargo, self.payload_kg = {}, 0.0
            self._activity("waiting")
            return True, f"delivered {units} units", units

        if action.kind == CHARGE:
            self._activity("charging")
            period = float(self.get_parameter("charge_check_period_s").value)
            while self.battery() < action.value and not self._abort.is_set():
                self.sim_sleep(period)
            self._activity("waiting")
            return not self._abort.is_set(), f"charged to {self.battery():.1f} %", 0

        if action.kind == WAIT:
            self._activity("charging" if self.location == CHARGER else "waiting")
            self.sim_sleep(action.value)
            self._activity("waiting")
            return not self._abort.is_set(), f"waited {action.value:g} s at {self.location}", 0
        return False, "unknown action", 0

    def _handle_unreachable(self, destination):
        """The goal was abandoned after the whole recovery ladder. Get the robot somewhere safe.

        Cargo stays on board (it is virtual and nothing was lost); a later DELIVER can still
        deliver it. If the charger - the safe place - cannot be reached either, the robot halts
        and the mission ends: it needs a human.
        """
        self.nav_stats["goals_abandoned"] += 1
        detail = f"could not reach {destination}"
        if self._abort.is_set():
            return detail + " (emergency stop)"
        if destination == CHARGER or not self.nav_cfg["return_to_charger_on_failure"]:
            if destination == CHARGER:
                self._declare_stuck()
                return detail + " - charger unreachable, robot HALTED (navigation_stuck)"
            return detail
        self.get_logger().warn(f"{detail}; returning to the charger")
        self._activity("driving")
        home, steps = self.navigate_to(CHARGER)
        self.recovery_log += ["return to charger"] + steps
        self._activity("waiting")
        if home:
            self.nav_stats["returned_to_charger"] += 1
            return detail + "; returned to the charger"
        self._declare_stuck()
        return detail + "; charger unreachable too, robot HALTED (navigation_stuck)"

    def _declare_stuck(self):
        self.stuck = True
        for _ in range(10):
            self.halt_pub.publish(Twist())
            time.sleep(0.05)
        self._activity("stopped")
        self.get_logger().error("NAVIGATION STUCK: the robot cannot reach the charger. "
                                "Halting and ending the mission - needs a human.")

    def _event(self, action, phase, detail, t0, duration, pred, distance, energy, charged,
               b0, b1, units):
        msg = TaskEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.run_id, msg.seq = self.run_id, self.seq
        msg.action, msg.target = action.kind, action.destination() or ""
        msg.phase, msg.detail = phase, detail
        msg.sim_time_s = t0
        msg.duration_s, msg.predicted_duration_s = float(duration), float(pred.duration_s)
        msg.distance_m, msg.predicted_distance_m = float(distance), float(pred.distance_m)
        msg.energy_wh, msg.predicted_energy_wh = float(energy), float(pred.energy_wh)
        msg.charged_wh = float(charged)
        msg.battery_start_percent, msg.battery_end_percent = float(b0), float(b1)
        msg.payload_kg, msg.units = float(self.payload_kg), int(units)
        self.event_pub.publish(msg)
        row = {
            "seq": self.seq, "action": action.kind, "target": msg.target, "phase": phase,
            "detail": detail, "sim_start_s": round(t0, 2), "duration_s": round(duration, 2),
            "predicted_duration_s": round(pred.duration_s, 2), "distance_m": round(distance, 3),
            "predicted_distance_m": round(pred.distance_m, 3), "energy_wh": round(energy, 4),
            "predicted_energy_wh": round(pred.energy_wh, 4), "charged_wh": round(charged, 4),
            "predicted_charged_wh": round(pred.charged_wh, 4),
            "battery_start_percent": round(b0, 3), "battery_end_percent": round(b1, 3),
            "predicted_battery_end_percent": round(pred.battery_end_percent, 3),
            "payload_kg": round(self.payload_kg, 3), "units": units, "location": self.location,
            "recovery": " | ".join(getattr(self, "recovery_log", [])) if phase != "STARTED" else "",
        }
        if phase != "STARTED":
            self.rows.append(row)
            self._csv.writerow(row)
            self._csv_fh.flush()
        return row

    # ------------------------------------------------------------------ run
    def _run(self):
        self.get_logger().info("waiting for Nav2, telemetry and the sections ...")
        if not self.wait_for_navigation_active():
            self.get_logger().error("Nav2 navigation never became active (bringup failed or hung) "
                                    "- mission aborted. Restart the launch.")
            return
        self.nav_client.wait_for_server()
        while self.telemetry is None:
            time.sleep(0.5)
        for sid, client in self.pickup_clients.items():
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"waiting for /factory/{sid}/pickup ...")
        if not self.publish_initial_pose():
            self.get_logger().error("AMCL never confirmed the initial pose - mission aborted")
            return
        self.sim_sleep(3.0)
        self.get_logger().info("localised; starting mission")
        self._activity("waiting")

        t_start, e_start = self.sim_now(), self.energy_counters()
        ok_count = 0
        for action in self.plan:
            if self._abort.is_set() or self.stuck:
                break
            ok_count += self.run_action(action)
        self.write_summary(t_start, e_start, ok_count)

    def write_summary(self, t_start, e_start, ok_count):
        e_end = self.energy_counters()
        done = [r for r in self.rows if r["phase"] == "SUCCEEDED"]

        def err(key):
            pairs = [(r[key], r[f"predicted_{key}"]) for r in done
                     if r[f"predicted_{key}"] > 1e-6 and r["action"] != CHARGE]
            if not pairs:
                return None
            return round(100.0 * (sum(m for m, _ in pairs) - sum(p for _, p in pairs))
                         / sum(p for _, p in pairs), 1)

        summary = {
            "run_id": self.run_id,
            "scenario": self.cfg["name"],
            "seed": self.cfg["time"]["seed"],
            "plan": "; ".join(str(a) for a in self.plan),
            "ended_by": ("emergency_stop" if self._abort.is_set()
                         else "navigation_stuck" if self.stuck else "plan_finished"),
            "actions": len(self.rows), "succeeded": ok_count,
            "failed": len(self.rows) - ok_count,
            "sim_duration_s": round(self.sim_now() - t_start, 1),
            "distance_m": round(self.distance_travelled, 2),
            "energy_drawn_wh": round(e_end[0] - e_start[0], 3),
            "energy_charged_wh": round(e_end[1] - e_start[1], 3),
            "battery_end_percent": round(self.battery(), 2),
            "delivered_units": self.delivered,
            "navigation": dict(self.nav_stats),
            "prediction_error_percent (non-charge actions, total measured vs predicted)": {
                "duration": err("duration_s"), "distance": err("distance_m"),
                "energy": err("energy_wh")},
        }
        with open(self.summary_path, "w") as fh:
            yaml.safe_dump(summary, fh, sort_keys=False)
        header = ("MISSION ENDED BY EMERGENCY STOP" if self._abort.is_set()
                  else "MISSION ENDED: NAVIGATION STUCK" if self.stuck else "MISSION COMPLETE")
        self.get_logger().info(header + "\n" + yaml.safe_dump(summary, sort_keys=False))

    def destroy_node(self):
        self._csv_fh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MissionExecutor()
    # Single-threaded is enough: the mission runs on its own worker thread and only waits on
    # futures/events, so the executor just has to deliver callbacks. The multi-threaded executor
    # used ~76 % of a core here, mostly handling /clock.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
