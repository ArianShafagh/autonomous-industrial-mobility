"""Terminal view of the three sections: `ros2 run robofetch_factory factory_monitor`.

Prints one line per section every second from /factory/<id>/status. Read-only.
"""
import math

import rclpy
from rclpy.node import Node

from robofetch_interfaces.msg import SectionStatus


class Monitor(Node):
    def __init__(self, sections=("A", "B", "C")):
        super().__init__("factory_monitor")
        self.latest = {}
        for sid in sections:
            self.create_subscription(SectionStatus, f"/factory/{sid}/status",
                                     lambda m, sid=sid: self.latest.__setitem__(sid, m), 10)
        self.create_timer(1.0, self._print)

    def _print(self):
        if not self.latest:
            print("waiting for /factory/*/status ...")
            return
        any_msg = next(iter(self.latest.values()))
        print(f"\nfactory time {any_msg.factory_time_s / 3600:6.2f} h  "
              f"(scenario {any_msg.scenario}, x{any_msg.time_scale:g})")
        print(" sec status    buffer   fill  produced picked  rate/h  health  to-full  "
              "fault-left lost")
        for sid in sorted(self.latest):
            m = self.latest[sid]
            ttf = "   inf" if math.isinf(m.time_to_full_s) else f"{m.time_to_full_s:6.0f}s"
            print(f"  {sid}  {m.status_text:9s} {m.buffer_units:3d}/{m.buffer_capacity:<3d} "
                  f"{100 * m.buffer_fill:5.0f}%  {m.produced_total:6d}  {m.picked_total:5d}  "
                  f"{m.rate_actual_per_hour:6.1f}  {m.health_percent:5.1f}%  {ttf}  "
                  f"{m.fault_remaining_s:7.0f}s  {m.lost_units:5.1f}", flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
