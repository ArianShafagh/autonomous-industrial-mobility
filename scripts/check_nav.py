#!/usr/bin/env python3
"""WP1 system check: can the robot localise and navigate the factory maze?

Run with the navigation stack up (ros2 launch robofetch_nav navigation.launch.py). It:
  1. sets the AMCL initial pose on the charger (where the robot spawns);
  2. asks the Nav2 planner for the path between every pair of POIs (no driving) and compares its
     length with the generated path_matrix.yaml;
  3. drives a tour charger -> A -> delivery -> B -> delivery -> C -> delivery -> charger and, per
     leg, records success, time, distance actually driven (Gazebo ground truth), final position
     error to the goal and AMCL localisation error.
Results are printed and written to logs/check_nav_<timestamp>.csv.

    venv/bin/python scripts/check_nav.py [--skip-tour]
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from robofetch_factory.layout import load_path_matrix, load_pois

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
TOUR = ["A", "delivery", "B", "delivery", "C", "delivery", "charger"]


def quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavCheck(Node):
    def __init__(self):
        super().__init__("check_nav", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.pois = load_pois()
        self.matrix = load_path_matrix()
        self.truth = None
        self.amcl = None
        self.create_subscription(Pose, "/model/robofetch/pose", self._on_truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.plan = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def _on_truth(self, msg):
        self.truth = (msg.position.x, msg.position.y)

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        self.amcl = (p.x, p.y)

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_future(self, future, timeout):
        return self.spin_until(future.done, timeout) and future.result()

    def pose_stamped(self, name):
        p = self.pois[name]
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = p["x"], p["y"]
        ps.pose.orientation.z, ps.pose.orientation.w = quat(p["yaw"])
        return ps

    # ---------------------------------------------------------------------------------------
    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        start = self.pose_stamped("charger")
        msg.pose.pose = start.pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            if self.spin_until(lambda: self.amcl is not None, 3.0):
                return True
        return False

    def planned_length(self, a, b):
        goal = ComputePathToPose.Goal()
        goal.start = self.pose_stamped(a)
        goal.goal = self.pose_stamped(b)
        goal.use_start = True
        handle = self.wait_future(self.plan.send_goal_async(goal), 10.0)
        if not handle or not handle.accepted:
            return None
        res = self.wait_future(handle.get_result_async(), 30.0)
        if not res or not res.result.path.poses:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))

    def drive(self, name, timeout=300.0):
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(name)
        driven = 0.0
        last = self.truth
        t0 = self.get_clock().now().nanoseconds / 1e9
        handle = self.wait_future(self.nav.send_goal_async(goal), 10.0)
        if not handle or not handle.accepted:
            return {"goal": name, "success": False, "detail": "goal rejected"}
        result = handle.get_result_async()
        end = time.monotonic() + timeout
        while not result.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.truth and last:
                driven += math.dist(self.truth, last)
            last = self.truth
        if not result.done():
            handle.cancel_goal_async()
            return {"goal": name, "success": False, "detail": "timeout"}
        self.spin_until(lambda: False, 1.0)   # let the final AMCL/truth messages arrive
        status = result.result().status      # 4 = SUCCEEDED
        p = self.pois[name]
        return {
            "goal": name,
            "success": status == 4,
            "detail": f"status {status}",
            "sim_time_s": round(self.get_clock().now().nanoseconds / 1e9 - t0, 1),
            "driven_m": round(driven, 2),
            "goal_error_m": round(math.dist(self.truth, (p["x"], p["y"])), 3),
            "amcl_error_m": round(math.dist(self.truth, self.amcl), 3),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-tour", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = NavCheck()
    rows = []
    print("[check] waiting for Nav2 action servers and ground-truth pose ...")
    if not (node.nav.wait_for_server(timeout_sec=120) and node.plan.wait_for_server(timeout_sec=60)):
        sys.exit("[check] Nav2 action servers not available")
    if not node.spin_until(lambda: node.truth is not None, 60):
        sys.exit("[check] no /model/robofetch/pose - is the bridge running?")
    if not node.set_initial_pose():
        sys.exit("[check] AMCL never published a pose")
    node.spin_until(lambda: False, 3.0)
    print(f"[check] localised: amcl={node.amcl} truth={node.truth}")

    print("\n[check] planner path length vs generated matrix")
    names = list(node.pois)
    worst = 0.0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            planned = node.planned_length(a, b)
            expected = node.matrix[a][b]
            if planned is None:
                print(f"  {a:>8s} -> {b:<8s}  NO PATH")
                rows.append({"kind": "plan", "goal": f"{a}->{b}", "success": False})
                continue
            diff = 100.0 * (planned - expected) / expected
            worst = max(worst, abs(diff))
            print(f"  {a:>8s} -> {b:<8s}  planner {planned:6.2f} m   matrix {expected:6.2f} m"
                  f"   diff {diff:+5.1f} %")
            rows.append({"kind": "plan", "goal": f"{a}->{b}", "success": True,
                         "planned_m": round(planned, 2), "matrix_m": expected,
                         "diff_pct": round(diff, 1)})
    print(f"  worst |diff| = {worst:.1f} %")

    if not args.skip_tour:
        print("\n[check] driving the tour", " -> ".join(["charger"] + TOUR))
        prev = "charger"
        for name in TOUR:
            r = node.drive(name)
            r.update({"kind": "drive", "from": prev, "matrix_m": node.matrix[prev][name]})
            rows.append(r)
            print(f"  {prev:>8s} -> {name:<8s} {r}")
            if not r["success"]:
                break
            prev = name

    os.makedirs(os.path.join(WS, "logs"), exist_ok=True)
    out = os.path.join(WS, "logs", time.strftime("check_nav_%Y%m%d_%H%M%S.csv"))
    keys = sorted({k for r in rows for k in r})
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"\n[check] wrote {out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
