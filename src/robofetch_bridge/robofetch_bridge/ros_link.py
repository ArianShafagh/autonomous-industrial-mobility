"""ROS 2 side of the monitor: listens, never commands.

    subscribes /factory/<id>/status   live production sections
               /robot/telemetry       battery, temperature, condition, payload
               /mission/decision      what the AI decided and why
               /mission/events        actions started and finished
               /amcl_pose             where the robot thinks it is

There is no publisher of any kind here. The dashboard is a window, not a control panel: the robot
runs the shift by itself, and the only way to intervene is `./scripts/stop.sh` at the console.

It runs on a background thread inside the FastAPI process, so an HTTP handler can read the latest
state as plain Python data.
"""
import collections
import json
import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robofetch_interfaces.msg import SectionStatus, TaskEvent

SECTIONS = ("A", "B", "C")
HISTORY = 400            # decisions and events kept for the page


def section_to_dict(msg):
    return {
        "section_id": msg.section_id, "status": msg.status_text, "scenario": msg.scenario,
        "produced_total": msg.produced_total, "picked_total": msg.picked_total,
        "buffer_units": msg.buffer_units, "buffer_capacity": msg.buffer_capacity,
        "buffer_fill": round(msg.buffer_fill, 3), "unit_mass_kg": round(msg.unit_mass_kg, 3),
        "rate_nominal_per_hour": round(msg.rate_nominal_per_hour, 1),
        "rate_actual_per_hour": round(msg.rate_actual_per_hour, 1),
        "health_percent": round(msg.health_percent, 1),
        "time_to_full_s": (None if msg.time_to_full_s != msg.time_to_full_s
                           or msg.time_to_full_s == float("inf") else round(msg.time_to_full_s)),
        "fault_remaining_s": round(msg.fault_remaining_s),
        "faults_total": msg.faults_total, "lost_units": round(msg.lost_units, 2),
        "blocked_time_s": round(msg.blocked_time_s, 1),
        "factory_time_s": round(msg.factory_time_s, 1),
    }


class RosLink(Node):
    def __init__(self):
        super().__init__("robofetch_monitor",
                         parameter_overrides=[rclpy.parameter.Parameter(
                             "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.sections = {}
        self.telemetry = {}
        self.robot_xy = None
        self.trail = collections.deque(maxlen=400)     # where the robot has been, for the map
        self.decisions = collections.deque(maxlen=HISTORY)
        self.events = collections.deque(maxlen=HISTORY)
        self.delivered = {sid: 0 for sid in SECTIONS}
        self.energy_history = collections.deque(maxlen=600)

        for sid in SECTIONS:
            self.create_subscription(SectionStatus, f"/factory/{sid}/status",
                                     self._on_section, 10)
        self.create_subscription(String, "/robot/telemetry", self._on_telemetry, 10)
        self.create_subscription(String, "/mission/decision", self._on_decision, 10)
        self.create_subscription(TaskEvent, "/mission/events", self._on_event, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        self.get_logger().info("monitor listening (read-only)")

    # ------------------------------------------------------------------ callbacks
    def _on_section(self, msg):
        with self.lock:
            self.sections[msg.section_id] = section_to_dict(msg)

    def _on_telemetry(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.telemetry = data
            self.energy_history.append({
                "t": data.get("ts"), "battery": data.get("battery_percent"),
                "energy_wh": data.get("cumulative_energy_wh"),
                "temperature_c": data.get("temperature_c")})

    def _on_decision(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.decisions.appendleft(data)

    def _on_event(self, msg):
        if msg.phase == "STARTED":
            return
        with self.lock:
            if msg.action == "DELIVER" and msg.phase == "SUCCEEDED":
                # Which section the units came from is in the executor's log; the monitor only
                # needs the total, so it counts what arrived.
                self.delivered["_total"] = self.delivered.get("_total", 0) + msg.units
            self.events.appendleft({
                "seq": msg.seq, "action": msg.action, "target": msg.target, "phase": msg.phase,
                "detail": msg.detail, "sim_time_s": round(msg.sim_time_s, 1),
                "duration_s": round(msg.duration_s, 1),
                "predicted_duration_s": round(msg.predicted_duration_s, 1),
                "distance_m": round(msg.distance_m, 2),
                "predicted_distance_m": round(msg.predicted_distance_m, 2),
                "energy_wh": round(msg.energy_wh, 3),
                "predicted_energy_wh": round(msg.predicted_energy_wh, 3),
                "battery_end_percent": round(msg.battery_end_percent, 1),
                "units": msg.units, "payload_kg": round(msg.payload_kg, 2),
            })

    def _on_pose(self, msg):
        with self.lock:
            self.robot_xy = (round(msg.pose.pose.position.x, 3),
                             round(msg.pose.pose.position.y, 3))
            # One trail point per ~0.15 m of travel: 400 points then cover ~60 m, a whole shift's
            # worth of route, without storing thousands of near-identical poses.
            last = self.trail[-1] if self.trail else None
            if last is None or abs(last[0] - self.robot_xy[0]) + abs(last[1] - self.robot_xy[1]) > 0.15:
                self.trail.append(self.robot_xy)

    # --------------------------------------------------------------------- reading
    def snapshot(self):
        """Everything the page shows, as plain data."""
        with self.lock:
            totals = {
                "delivered_units": self.delivered.get("_total", 0),
                "produced_units": sum(s["produced_total"] for s in self.sections.values()),
                "lost_units": round(sum(s["lost_units"] for s in self.sections.values()), 2),
                "energy_wh": round(self.telemetry.get("cumulative_energy_wh", 0.0), 3),
                "distance_m": round(self.telemetry.get("cumulative_distance_m", 0.0), 1),
            }
            return {
                "sections": [self.sections[s] for s in SECTIONS if s in self.sections],
                "telemetry": dict(self.telemetry),
                "robot_xy": self.robot_xy,
                "trail": list(self.trail),
                "decisions": list(self.decisions)[:25],
                "events": list(self.events)[:25],
                "totals": totals,
                "energy_history": list(self.energy_history)[-120:],
                "connected": bool(self.sections) or bool(self.telemetry),
            }


class RosThread:
    """Runs the monitor node on its own thread inside the web process."""

    def __init__(self):
        rclpy.init()
        self.node = RosLink()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def snapshot(self):
        return self.node.snapshot()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
