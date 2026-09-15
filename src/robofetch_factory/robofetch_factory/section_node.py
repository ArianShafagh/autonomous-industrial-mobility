"""One live production section (A, B or C).

Runs `factory_model.Section` against the simulation clock and exposes it to the robot and AI:

    publishes  /factory/<id>/status   robofetch_interfaces/SectionStatus  (publish_rate Hz)
    service    /factory/<id>/pickup   robofetch_interfaces/Pickup         (robot collects units)
    writes     <log_dir>/<run_id>_section_<id>.csv                        (one row per publish)

The model is stepped on every timer tick with the elapsed SIM time, so pausing or slowing
Gazebo pauses or slows the factory too, and results do not depend on the tick rate.

    ros2 run robofetch_factory section_node --ros-args -p section_id:=B -p scenario:=balanced
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from robofetch_factory.factory_model import Section, load_config, section_params
from robofetch_interfaces.msg import SectionStatus
from robofetch_interfaces.srv import Pickup

CSV_FIELDS = ["sim_time_s", "section_id", "status", "factory_time_s", "produced_total",
              "picked_total", "buffer_units", "buffer_capacity", "buffer_fill",
              "rate_actual_per_hour", "health_percent", "time_to_full_s",
              "fault_remaining_s", "faults_total", "lost_units", "blocked_time_s"]


def default_log_dir():
    # <ws>/src/robofetch_factory/robofetch_factory/section_node.py -> <ws>/logs (symlink install)
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.normpath(os.path.join(here, "..", "..", ".."))
    return os.path.join(ws, "logs") if os.path.isdir(os.path.join(ws, "src")) else \
        os.path.join(os.path.expanduser("~"), ".ros", "robofetch_logs")


class SectionNode(Node):
    def __init__(self):
        super().__init__("section_node")
        self.declare_parameter("section_id", "A")
        self.declare_parameter("scenario", "balanced")
        self.declare_parameter("seed", -1)                 # -1 = the scenario's seed
        self.declare_parameter("step_period", 0.1)         # sim seconds between model steps
        self.declare_parameter("publish_rate", 1.0)        # Hz
        self.declare_parameter("log_dir", default_log_dir())
        self.declare_parameter("run_id", "")

        self.sid = self.get_parameter("section_id").value
        self.scenario = self.get_parameter("scenario").value
        cfg = load_config(self.scenario)
        seed = int(self.get_parameter("seed").value)
        seed = cfg["time"].get("seed", 0) if seed < 0 else seed
        if self.sid not in cfg["factory"]["sections"]:
            raise SystemExit(f"section '{self.sid}' not in scenario '{self.scenario}'")
        self.time_scale = float(cfg["time"]["time_scale"])
        self.section = Section(self.sid, section_params(cfg, self.sid), self.time_scale, seed)

        self.pub = self.create_publisher(SectionStatus, f"/factory/{self.sid}/status", 10)
        self.create_service(Pickup, f"/factory/{self.sid}/pickup", self._on_pickup)

        self._last_sim = None
        self.create_timer(float(self.get_parameter("step_period").value), self._tick)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

        run_id = self.get_parameter("run_id").value or time.strftime("run_%Y%m%d_%H%M%S")
        log_dir = self.get_parameter("log_dir").value
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f"{run_id}_section_{self.sid}.csv")
        self._csv_fh = open(self.csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
        self._csv.writeheader()

        p = self.section.params
        self.get_logger().info(
            f"section {self.sid} [{self.scenario}, seed {seed}]: {p.rate_per_hour:g} units/h, "
            f"buffer {p.buffer_capacity}, {p.unit_mass_kg} kg/unit, time_scale {self.time_scale:g}"
            f" -> logging {self.csv_path}")

    def _sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        now = self._sim_now()
        if self._last_sim is None or now < self._last_sim:   # first tick, or sim clock reset
            self._last_sim = now
            return
        self.section.step(now - self._last_sim)
        self._last_sim = now

    def _publish(self):
        if self._last_sim is None:
            return
        snap = self.section.snapshot()
        msg = SectionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.section_id = self.sid
        msg.scenario = self.scenario
        msg.status = snap["status_code"]
        msg.status_text = snap["status"]
        msg.time_scale = self.time_scale
        msg.factory_time_s = snap["factory_time_s"]
        for key in ("produced_total", "picked_total", "buffer_units", "buffer_capacity",
                    "faults_total"):
            setattr(msg, key, int(snap[key]))
        for key in ("buffer_fill", "unit_mass_kg", "buffer_mass_kg", "rate_nominal_per_hour",
                    "rate_actual_per_hour", "health_percent", "time_to_full_s",
                    "fault_remaining_s", "lost_units", "blocked_time_s"):
            setattr(msg, key, float(snap[key]))
        self.pub.publish(msg)

        row = dict(snap, sim_time_s=round(self._last_sim, 2))
        for key, value in row.items():
            if isinstance(value, float) and math.isfinite(value):
                row[key] = round(value, 4)
        self._csv.writerow(row)
        self._csv_fh.flush()

    def _on_pickup(self, request, response):
        max_units = None if request.max_units < 0 else request.max_units
        max_mass = None if request.max_mass_kg <= 0.0 else request.max_mass_kg
        units = self.section.pickup(max_units, max_mass)
        response.units = units
        response.mass_kg = float(units * self.section.params.unit_mass_kg)
        response.buffer_left = self.section.buffer
        response.success = units > 0
        response.message = (f"took {units} units ({response.mass_kg:.2f} kg), "
                            f"{self.section.buffer} left" if units
                            else f"nothing taken, {self.section.buffer} left")
        self.get_logger().info(f"pickup: {response.message}")
        return response

    def destroy_node(self):
        self._csv_fh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
