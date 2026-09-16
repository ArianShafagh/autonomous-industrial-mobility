"""The live production sections (A, B, C) in ONE process.

Each section keeps its own interface, exactly as if it were a separate machine controller:

    publishes  /factory/<id>/status   robofetch_interfaces/SectionStatus  (publish_rate Hz)
    service    /factory/<id>/pickup   robofetch_interfaces/Pickup         (robot collects units)
    writes     <log_dir>/<run_id>_section_<id>.csv                        (one row per publish)

Why one process and not one node per section: every ROS node with use_sim_time processes every
/clock message from Gazebo, and in Python that cost ~55 % of a CPU core PER NODE. Three section
nodes plus the robot model starved Nav2's start-up until its lifecycle transitions timed out
(WP3). One process = one /clock subscriber.

The models are stepped on the SIM clock, so pausing or slowing Gazebo pauses the factory too.

    ros2 run robofetch_factory factory_node --ros-args -p scenario:=balanced -p use_sim_time:=true
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from robofetch_factory.factory_model import build_sections, load_config
from robofetch_interfaces.msg import SectionStatus
from robofetch_interfaces.srv import Pickup

CSV_FIELDS = ["sim_time_s", "section_id", "status", "factory_time_s", "produced_total",
              "picked_total", "buffer_units", "buffer_capacity", "buffer_fill",
              "rate_actual_per_hour", "health_percent", "time_to_full_s",
              "fault_remaining_s", "faults_total", "lost_units", "blocked_time_s"]


def default_log_dir():
    # <ws>/src/robofetch_factory/robofetch_factory/factory_node.py -> <ws>/logs (symlink install)
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.normpath(os.path.join(here, "..", "..", ".."))
    if os.path.isdir(os.path.join(ws, "src")):
        return os.path.join(ws, "logs")
    return os.path.join(os.path.expanduser("~"), ".ros", "robofetch_logs")


class FactoryNode(Node):
    def __init__(self):
        super().__init__("factory")
        self.declare_parameter("scenario", "balanced")
        self.declare_parameter("seed", -1)                 # -1 = the scenario's seed
        self.declare_parameter("step_period", 0.1)         # sim seconds between model steps
        self.declare_parameter("publish_rate", 1.0)        # Hz
        self.declare_parameter("log_dir", default_log_dir())
        self.declare_parameter("run_id", "")

        self.scenario = self.get_parameter("scenario").value
        cfg = load_config(self.scenario)
        seed = int(self.get_parameter("seed").value)
        seed = cfg["time"].get("seed", 0) if seed < 0 else seed
        self.time_scale = float(cfg["time"]["time_scale"])
        self.sections = build_sections(cfg, seed)

        self.pubs = {}
        for sid in self.sections:
            self.pubs[sid] = self.create_publisher(SectionStatus, f"/factory/{sid}/status", 10)
            self.create_service(Pickup, f"/factory/{sid}/pickup",
                                lambda req, res, sid=sid: self._on_pickup(sid, req, res))

        self._last_sim = None
        self.create_timer(float(self.get_parameter("step_period").value), self._tick)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

        run_id = self.get_parameter("run_id").value or time.strftime("run_%Y%m%d_%H%M%S")
        log_dir = self.get_parameter("log_dir").value
        os.makedirs(log_dir, exist_ok=True)
        self._csv_fh, self._csv = {}, {}
        for sid in self.sections:
            path = os.path.join(log_dir, f"{run_id}_section_{sid}.csv")
            self._csv_fh[sid] = open(path, "w", newline="")
            self._csv[sid] = csv.DictWriter(self._csv_fh[sid], fieldnames=CSV_FIELDS,
                                            extrasaction="ignore")
            self._csv[sid].writeheader()

        for sid, s in self.sections.items():
            p = s.params
            self.get_logger().info(
                f"section {sid} [{self.scenario}, seed {seed}]: {p.rate_per_hour:g} units/h, "
                f"buffer {p.buffer_capacity}, {p.unit_mass_kg} kg/unit, time_scale {self.time_scale:g}")
        self.get_logger().info(f"logging to {log_dir}/{run_id}_section_*.csv")

    def _sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        now = self._sim_now()
        if self._last_sim is None or now < self._last_sim:   # first tick, or sim clock reset
            self._last_sim = now
            return
        for s in self.sections.values():
            s.step(now - self._last_sim)
        self._last_sim = now

    def _publish(self):
        if self._last_sim is None:
            return
        for sid, section in self.sections.items():
            snap = section.snapshot()
            msg = SectionStatus()
            msg.stamp = self.get_clock().now().to_msg()
            msg.section_id = sid
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
            self.pubs[sid].publish(msg)

            row = dict(snap, sim_time_s=round(self._last_sim, 2))
            for key, value in row.items():
                if isinstance(value, float) and math.isfinite(value):
                    row[key] = round(value, 4)
            self._csv[sid].writerow(row)
            self._csv_fh[sid].flush()

    def _on_pickup(self, sid, request, response):
        section = self.sections[sid]
        max_units = None if request.max_units < 0 else request.max_units
        max_mass = None if request.max_mass_kg <= 0.0 else request.max_mass_kg
        units = section.pickup(max_units, max_mass)
        response.units = units
        response.mass_kg = float(units * section.params.unit_mass_kg)
        response.buffer_left = section.buffer
        response.success = units > 0
        response.message = (f"took {units} units ({response.mass_kg:.2f} kg), {section.buffer} left"
                            if units else f"nothing taken, {section.buffer} left")
        self.get_logger().info(f"pickup {sid}: {response.message}")
        return response

    def destroy_node(self):
        for fh in self._csv_fh.values():
            fh.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = FactoryNode()
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
