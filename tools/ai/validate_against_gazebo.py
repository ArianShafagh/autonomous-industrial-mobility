#!/usr/bin/env python3
"""Does the fast simulator agree with the real Gazebo runs?

Replays the action sequence of a finished Gazebo mission in the fast simulator and compares
distance, duration and energy - per action and in total. If these drift apart, everything the AI
learns in the fast simulator becomes untrustworthy, so this is run after every change to either.

    venv/bin/python tools/ai/validate_against_gazebo.py                 # newest mission log
    venv/bin/python tools/ai/validate_against_gazebo.py --csv logs/run_..._mission.csv
"""
import argparse
import csv
import glob
import os
import sys

import yaml

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

from robofetch_core.mission_plan import parse_action  # noqa: E402
from robofetch_ai.env.factory_sim import FactorySim  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")


def action_from_row(row):
    kind, target = row["action"], row["target"]
    if kind == "PICKUP":
        return parse_action(f"PICKUP:{target}")
    if kind == "CHARGE":
        return parse_action(f"CHARGE:{row['predicted_battery_end_percent']}")
    if kind == "WAIT":
        return parse_action(f"WAIT:{max(1.0, float(row['duration_s']))}")
    return parse_action(kind)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None, help="a logs/<run_id>_mission.csv (default: newest)")
    ap.add_argument("--scenario", default=None, help="override the scenario of the run")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    path = args.csv or max(glob.glob(os.path.join(WS, "logs", "*_mission.csv")), key=os.path.getmtime)
    summary_path = path.replace("_mission.csv", "_mission_summary.yaml")
    summary = yaml.safe_load(open(summary_path)) if os.path.exists(summary_path) else {}
    scenario = args.scenario or summary.get("scenario", "balanced")
    seed = args.seed if args.seed is not None else summary.get("seed", 1)
    rows = [r for r in csv.DictReader(open(path)) if r["phase"] == "SUCCEEDED"]
    print(f"replaying {len(rows)} actions of {os.path.basename(path)} "
          f"(scenario {scenario}, seed {seed})\n")

    sim = FactorySim(scenario, seed, CONFIG_DIR, shift_duration_s=1e9)
    print(f"{'#':>2} {'action':<12}{'distance m (gz/sim)':>24}{'duration s (gz/sim)':>22}"
          f"{'energy Wh (gz/sim)':>24}")
    totals = {k: [0.0, 0.0] for k in ("distance", "duration", "energy")}
    for i, row in enumerate(rows, 1):
        outcome = sim.step(action_from_row(row))
        gz = (float(row["distance_m"]), float(row["duration_s"]), float(row["energy_wh"]))
        fs = (outcome.distance_m, outcome.duration_s, outcome.energy_wh)
        for key, a, b in zip(totals, gz, fs):
            totals[key][0] += a
            totals[key][1] += b
        print(f"{i:2d} {row['action'] + ':' + row['target']:<12}"
              f"{gz[0]:11.2f} /{fs[0]:8.2f}  {gz[1]:10.1f} /{fs[1]:8.1f}  "
              f"{gz[2]:12.3f} /{fs[2]:8.3f}")

    print()
    worst = 0.0
    for key, (gz, fs) in totals.items():
        diff = 100.0 * (fs - gz) / gz if gz else 0.0
        worst = max(worst, abs(diff))
        print(f"total {key:9s} Gazebo {gz:9.2f}   fast sim {fs:9.2f}   diff {diff:+6.1f} %")
    print(f"\nworst total difference {worst:.1f} % "
          f"({'OK' if worst <= 15 else 'TOO LARGE - the fast simulator does not match the robot'})")
    return 0 if worst <= 15 else 1


if __name__ == "__main__":
    sys.exit(main())
