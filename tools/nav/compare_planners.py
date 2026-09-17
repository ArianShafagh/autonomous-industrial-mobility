#!/usr/bin/env python3
"""WP8 - which global path planner gives the shortest, cheapest, safest routes in the maze?

The planner is isolated from everything else: each planner computes the path, and the SAME
controller (MPPI) then follows it with FollowPath. Any difference in driving time or energy is
therefore caused by the path itself, not by different navigation behaviour.

Phase 1  plan only (no driving): every ordered pair of points of interest, several repeats.
         -> success, planning time, path length (vs the generated shortest-path matrix),
            total turning (smoothness), clearance to the nearest obstacle (safety margin)
Phase 2  drive: the tour charger -> A -> delivery -> B -> delivery -> C -> delivery -> charger
         with each planner's paths.
         -> success, driving time, ground-truth distance, energy from the robot model,
            final position error

Needs the navigation stack and the robot model running:
    ros2 launch robofetch_nav navigation.launch.py rviz:=false gz_extra:="-s --headless-rendering"
    ros2 run robofetch_core robot_state_node --ros-args -p use_sim_time:=true
    venv/bin/python -u tools/nav/compare_planners.py [--repeats 3] [--skip-drive]

Writes tools/nav/results/planners_plan_<time>.csv, planners_drive_<time>.csv and prints the tables.
"""
import argparse
import csv
import functools
import json
import math
import os
import statistics
import sys
import time

print = functools.partial(print, flush=True)  # noqa: A001

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "robofetch_factory"))

import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose, FollowPath  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

import generate_world  # noqa: E402  (the same map geometry the world and the map come from)
from robofetch_factory.layout import load_path_matrix, load_pois  # noqa: E402

PLANNERS = ["NavfnDijkstra", "NavfnAStar", "Smac2D", "ThetaStar", "SmacLattice"]
TOUR = ["A", "delivery", "B", "delivery", "C", "delivery", "charger"]
RESULTS = os.path.join(WS, "tools", "nav", "results")


def quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)


class Clearance:
    """Distance from any map point to the nearest wall or machine, from the generated grid."""

    def __init__(self):
        layout = generate_world.Layout(generate_world.LAYOUT)
        self.grid = generate_world.FineGrid(layout)

    def at(self, x, y):
        r, c = self.grid.to_px(x, y)
        if 0 <= r < self.grid.h and 0 <= c < self.grid.w:
            return float(self.grid.clearance[r, c])
        return 0.0


def path_metrics(poses, clearance):
    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    # Smoothness: total heading change along the path, from segments at least 5 cm long
    # (per-cell jitter of grid planners would otherwise dominate).
    headings, last = [], pts[0] if pts else None
    for p in pts[1:]:
        if math.dist(p, last) >= 0.05:
            headings.append(math.atan2(p[1] - last[1], p[0] - last[0]))
            last = p
    turning = 0.0
    for a, b in zip(headings, headings[1:]):
        turning += abs(math.atan2(math.sin(b - a), math.cos(b - a)))
    clear = [clearance.at(x, y) for x, y in pts] or [0.0]
    return length, turning, min(clear), statistics.fmean(clear)


class PlannerBench(Node):
    def __init__(self):
        super().__init__("planner_bench", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.pois = load_pois()
        self.matrix = load_path_matrix()
        self.plan_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.follow_client = ActionClient(self, FollowPath, "follow_path")
        self.truth = None
        self.telemetry = {}
        self.create_subscription(Pose, "/model/robofetch/pose", self._on_truth, 10)
        self.create_subscription(String, "/robot/telemetry", self._on_telemetry, 10)

    def _on_truth(self, msg):
        self.truth = (msg.position.x, msg.position.y)

    def _on_telemetry(self, msg):
        try:
            self.telemetry = json.loads(msg.data)
        except ValueError:
            pass

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def wait(self, future, timeout):
        return future.result() if self.spin_until(future.done, timeout) else None

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose(self, name):
        p = self.pois[name]
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = p["x"], p["y"]
        ps.pose.orientation.z, ps.pose.orientation.w = quat(p["yaw"])
        return ps

    def plan(self, planner, goal, start=None):
        request = ComputePathToPose.Goal()
        request.goal = self.pose(goal)
        request.planner_id = planner
        if start is not None:
            request.start, request.use_start = self.pose(start), True
        wall = time.perf_counter()
        handle = self.wait(self.plan_client.send_goal_async(request), 15.0)
        if handle is None or not handle.accepted:
            return None, "rejected", 0.0
        result = self.wait(handle.get_result_async(), 60.0)
        wall_ms = (time.perf_counter() - wall) * 1000.0
        if result is None:
            return None, "timeout", wall_ms
        r = result.result
        if r.error_code or not r.path.poses:
            return None, f"error {r.error_code} {r.error_msg}".strip(), wall_ms
        planning_ms = r.planning_time.sec * 1000.0 + r.planning_time.nanosec / 1e6
        return r.path, "ok", planning_ms or wall_ms

    def follow(self, path, timeout=300.0):
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = "FollowPath"
        handle = self.wait(self.follow_client.send_goal_async(request), 15.0)
        if handle is None or not handle.accepted:
            return False, "rejected"
        future = handle.get_result_async()
        driven, last = 0.0, self.truth
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth and last:
                driven += math.dist(self.truth, last)
            last = self.truth
        if not future.done():
            handle.cancel_goal_async()
            return False, "timeout", driven
        result = future.result()
        return result.status == 4 and not result.result.error_code, \
            f"status {result.status} error {result.result.error_code}", driven


def phase_plan(bench, clearance, repeats):
    rows = []
    names = list(bench.pois)
    pairs = [(a, b) for a in names for b in names if a != b]
    for planner in PLANNERS:
        for a, b in pairs:
            for rep in range(repeats):
                path, status, ms = bench.plan(planner, b, start=a)
                row = {"planner": planner, "from": a, "to": b, "repeat": rep, "status": status,
                       "planning_ms": round(ms, 2), "matrix_m": bench.matrix[a][b]}
                if path is not None:
                    length, turning, cmin, cmean = path_metrics(path.poses, clearance)
                    row.update(length_m=round(length, 3), turning_rad=round(turning, 3),
                               min_clearance_m=round(cmin, 3), mean_clearance_m=round(cmean, 3),
                               poses=len(path.poses))
                rows.append(row)
        ok = [r for r in rows if r["planner"] == planner and r["status"] == "ok"]
        print(f"  {planner:14s} planned {len(ok)}/{len(pairs) * repeats}")
    return rows


def phase_drive(bench, clearance, repeat=0):
    rows = []
    # Rotate the order every repeat, so slow drift over a long session (battery level, motor
    # temperature, simulator load) cannot systematically favour whichever planner goes first.
    order = PLANNERS[repeat % len(PLANNERS):] + PLANNERS[:repeat % len(PLANNERS)]
    for planner in order:
        print(f"  [repeat {repeat + 1}] driving the tour with {planner} ...")
        where = "charger"
        for goal in TOUR:
            path, status, ms = bench.plan(planner, goal)
            row = {"planner": planner, "repeat": repeat, "from": where, "to": goal,
                   "planning_ms": round(ms, 2), "matrix_m": bench.matrix[where][goal]}
            if path is None:
                row.update(success=False, detail=f"planning {status}")
                rows.append(row)
                print(f"    {where} -> {goal}: planning failed ({status})")
                break
            length, turning, cmin, _ = path_metrics(path.poses, clearance)
            e0 = float(bench.telemetry.get("cumulative_energy_wh", 0.0))
            t0 = bench.sim_now()
            ok, detail, driven = bench.follow(path)
            bench.spin_until(lambda: False, 1.5)          # let the next telemetry sample arrive
            p = bench.pois[goal]
            row.update(success=ok, detail=detail, planned_m=round(length, 3),
                       turning_rad=round(turning, 3), min_clearance_m=round(cmin, 3),
                       driven_m=round(driven, 3), time_s=round(bench.sim_now() - t0, 2),
                       energy_wh=round(float(bench.telemetry.get("cumulative_energy_wh", 0.0))
                                       - e0, 4),
                       goal_error_m=round(math.dist(bench.truth, (p["x"], p["y"])), 3)
                       if bench.truth else None)
            rows.append(row)
            print(f"    {where:>8s} -> {goal:<8s} {'ok ' if ok else 'FAIL'} planned {length:5.2f} m, "
                  f"driven {driven:5.2f} m, {row['time_s']:5.1f} s, {row['energy_wh']:.3f} Wh")
            if not ok:
                break
            where = goal
    return rows


def write(rows, name):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, time.strftime(f"{name}_%Y%m%d_%H%M%S.csv"))
    keys = sorted({k for r in rows for k in r})
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    return path


def mean(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    return statistics.fmean(vals) if vals else float("nan")


def report(plan_rows, drive_rows):
    print("\nPHASE 1 - planning only (all ordered POI pairs)")
    print(f"{'planner':14s} {'success':>8s} {'plan ms':>9s} {'ms p95':>8s} {'length m':>9s} "
          f"{'vs shortest':>11s} {'turning rad':>12s} {'min clear m':>12s} {'mean clear m':>13s}")
    for planner in PLANNERS:
        rows = [r for r in plan_rows if r["planner"] == planner]
        ok = [r for r in rows if r["status"] == "ok"]
        if not ok:
            print(f"{planner:14s} {'0/' + str(len(rows)):>8s}")
            continue
        ms = sorted(r["planning_ms"] for r in ok)
        excess = statistics.fmean(100.0 * (r["length_m"] - r["matrix_m"]) / r["matrix_m"] for r in ok)
        print(f"{planner:14s} {len(ok):>4d}/{len(rows):<3d} {statistics.fmean(ms):9.1f} "
              f"{ms[int(0.95 * (len(ms) - 1))]:8.1f} {mean(ok, 'length_m'):9.2f} {excess:+10.1f}% "
              f"{mean(ok, 'turning_rad'):12.2f} {min(r['min_clearance_m'] for r in ok):12.2f} "
              f"{mean(ok, 'mean_clearance_m'):13.2f}")
    if not drive_rows:
        return
    repeats = sorted({r.get("repeat", 0) for r in drive_rows})
    if len(repeats) > 1:
        report_repeats(drive_rows, repeats)
        return
    print("\nPHASE 2 - driving the tour (7 legs, same controller)")
    print(f"{'planner':14s} {'legs ok':>8s} {'time s':>8s} {'driven m':>9s} {'energy Wh':>10s} "
          f"{'Wh per km':>10s} {'turning rad':>12s} {'goal err m':>11s}")
    for planner in PLANNERS:
        rows = [r for r in drive_rows if r["planner"] == planner]
        ok = [r for r in rows if r.get("success")]
        total_time = sum(r["time_s"] for r in ok)
        total_m = sum(r["driven_m"] for r in ok)
        total_wh = sum(r["energy_wh"] for r in ok)
        print(f"{planner:14s} {len(ok):>4d}/{len(TOUR):<3d} {total_time:8.1f} {total_m:9.2f} "
              f"{total_wh:10.3f} {1000.0 * total_wh / total_m if total_m else float('nan'):10.2f} "
              f"{sum(r['turning_rad'] for r in ok):12.2f} {mean(ok, 'goal_error_m'):11.3f}")


def tour_totals(drive_rows, planner, repeat):
    """Totals of one complete tour, or None if any leg failed (a failed tour is not averaged)."""
    legs = [r for r in drive_rows if r["planner"] == planner and r.get("repeat", 0) == repeat]
    if len(legs) != len(TOUR) or not all(r.get("success") for r in legs):
        return None
    return {"time_s": sum(r["time_s"] for r in legs), "driven_m": sum(r["driven_m"] for r in legs),
            "energy_wh": sum(r["energy_wh"] for r in legs),
            "turning_rad": sum(r["turning_rad"] for r in legs),
            "goal_error_m": statistics.fmean(r["goal_error_m"] for r in legs)}


def ci95(values):
    """Mean and half-width of the 95 % confidence interval (Student t)."""
    from scipy import stats
    if len(values) < 2:
        return statistics.fmean(values), float("nan")
    half = stats.t.ppf(0.975, len(values) - 1) * statistics.stdev(values) / math.sqrt(len(values))
    return statistics.fmean(values), half


def report_repeats(drive_rows, repeats):
    from scipy import stats
    print(f"\nPHASE 2 - driving the tour, {len(repeats)} repeats per planner "
          "(mean +- 95 % CI of the tour totals)")
    header = (f"{'planner':14s} {'tours ok':>9s} {'time s':>15s} {'driven m':>14s} "
              f"{'energy Wh':>16s} {'turning rad':>14s} {'goal err m':>14s}")
    print(header)
    totals = {}
    for planner in PLANNERS:
        tours = [t for t in (tour_totals(drive_rows, planner, r) for r in repeats) if t]
        totals[planner] = {r: tour_totals(drive_rows, planner, r) for r in repeats}
        if not tours:
            print(f"{planner:14s} {'0/' + str(len(repeats)):>9s}")
            continue
        cells = []
        for key, fmt in (("time_s", "{:6.1f}"), ("driven_m", "{:5.2f}"), ("energy_wh", "{:6.3f}"),
                         ("turning_rad", "{:5.1f}"), ("goal_error_m", "{:5.3f}")):
            m, h = ci95([t[key] for t in tours])
            cells.append(f"{fmt.format(m)} +- {fmt.format(h).strip()}")
        print(f"{planner:14s} {len(tours):>5d}/{len(repeats):<3d} " + " ".join(f"{c:>15s}" for c in cells))

    base = "NavfnDijkstra"
    print(f"\npaired against the current default ({base}), same repeat = same session conditions:")
    print(f"{'planner':14s} {'time diff s':>18s} {'p (time)':>9s} {'energy diff Wh':>20s} {'p (energy)':>11s}")
    for planner in PLANNERS:
        if planner == base:
            continue
        pairs = [(totals[planner][r], totals[base][r]) for r in repeats
                 if totals[planner][r] and totals[base][r]]
        if len(pairs) < 2:
            print(f"{planner:14s}  not enough complete tours")
            continue
        dt = [a["time_s"] - b["time_s"] for a, b in pairs]
        de = [a["energy_wh"] - b["energy_wh"] for a, b in pairs]
        mt, ht = ci95(dt)
        me, he = ci95(de)
        pt = stats.ttest_rel([a["time_s"] for a, _ in pairs], [b["time_s"] for _, b in pairs]).pvalue
        pe = stats.ttest_rel([a["energy_wh"] for a, _ in pairs],
                             [b["energy_wh"] for _, b in pairs]).pvalue
        print(f"{planner:14s} {mt:+7.1f} +- {ht:5.1f} {pt:9.3f} {me:+9.4f} +- {he:7.4f} {pe:11.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--skip-drive", action="store_true")
    ap.add_argument("--skip-plan", action="store_true")
    ap.add_argument("--drive-repeats", type=int, default=1)
    args = ap.parse_args()

    rclpy.init()
    bench = PlannerBench()
    clearance = Clearance()

    print("waiting for Nav2 to be active ...")
    active = bench.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
    deadline = time.monotonic() + 300
    while time.monotonic() < deadline:
        if active.wait_for_service(timeout_sec=2.0):
            res = bench.wait(active.call_async(Trigger.Request()), 5.0)
            if res and res.success:
                break
        time.sleep(2.0)
    else:
        sys.exit("Nav2 navigation never became active")
    bench.plan_client.wait_for_server(timeout_sec=60)
    bench.follow_client.wait_for_server(timeout_sec=60)
    bench.spin_until(lambda: bench.truth is not None and bench.telemetry, 60)
    bench.spin_until(lambda: False, 3.0)

    plan_rows, plan_csv = [], None
    if not args.skip_plan:
        print(f"\nphase 1: planning {len(PLANNERS)} planners x 20 routes x {args.repeats} repeats")
        plan_rows = phase_plan(bench, clearance, args.repeats)
        plan_csv = write(plan_rows, "planners_plan")

    drive_rows = []
    if not args.skip_drive:
        print(f"\nphase 2: driving the tour with each planner, {args.drive_repeats} repeat(s)")
        for repeat in range(args.drive_repeats):
            drive_rows += phase_drive(bench, clearance, repeat)
            drive_csv = write(drive_rows, "planners_drive")      # saved after every repeat

    if plan_rows:
        report(plan_rows, drive_rows)
    elif drive_rows:
        report_repeats(drive_rows, sorted({r["repeat"] for r in drive_rows})) \
            if args.drive_repeats > 1 else report([], drive_rows)
    if plan_csv:
        print(f"\n-> {os.path.relpath(plan_csv, WS)}")
    if drive_rows:
        print(f"-> {os.path.relpath(drive_csv, WS)}")
    bench.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
