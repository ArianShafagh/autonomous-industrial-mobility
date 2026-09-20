#!/usr/bin/env python3
"""WP9 tier 2: whole shifts in Gazebo, each compared with the fast simulator on the same seed.

    venv/bin/python -u tools/ai/run_gazebo_batch.py                        # defaults below
    venv/bin/python -u tools/ai/run_gazebo_batch.py --models ns ppo --scenarios balanced \\
        --seeds 5000 5001 --shift-min 20
    venv/bin/python -u tools/ai/run_gazebo_batch.py --report-only tools/ai/results/gazebo_<ts>.csv

For every (model, scenario, seed): start the complete system headless through scripts/run.sh
(planner and everything else from config/run.yaml), wait for the mission summary, stop THIS run,
then play the same policy, scenario, seed and shift length in the fast simulator. The score is
computed the same way for both, from the Gazebo logs:
    delivered (mission summary) - lost production (section CSVs) - 0.5 x Wh drawn
    - 20 x safety violations (battery below reserve after an action away from the charger,
      temperature at the limit, battery empty - the fast simulator's rules)

Refuses to start while any simulation is running; results are written after every run, so an
interrupted batch keeps what it finished (resume with --skip-done FILE).
"""
import argparse
import csv
import glob
import os
import signal
import statistics
import subprocess
import sys
import time

import yaml

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

from robofetch_factory.factory_model import load_config  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")
RESULTS_DIR = os.path.join(WS, "tools", "ai", "results")
LOG_DIR = os.path.join(WS, "logs")
FIELDS = ["model", "scenario", "seed", "source", "score", "delivered_units", "lost_units",
          "energy_wh", "wh_per_unit", "distance_m", "charge_trips", "min_battery_percent",
          "violation_count", "actions", "failed_actions", "fallback_decisions", "nav_failures",
          "shift_s", "wall_s", "run_id"]


def simulation_running():
    out = subprocess.run(["bash", os.path.join(WS, "scripts", "stop.sh"), "--running"],
                         capture_output=True, text=True)
    return out.returncode == 0


def read_csv(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def gazebo_metrics(summary_path, cfg):
    with open(summary_path) as fh:
        s = yaml.safe_load(fh)
    run_id = s["run_id"]
    o = cfg["mission"]["objective"]
    reserve = float(cfg["robot"]["battery"]["reserve_percent"])
    max_c = float(cfg["robot"]["thermal"]["max_c"])

    lost = 0.0
    for path in glob.glob(os.path.join(LOG_DIR, f"{run_id}_section_*.csv")):
        rows = read_csv(path)
        if rows:
            lost += float(rows[-1]["lost_units"])
    actions = read_csv(os.path.join(LOG_DIR, f"{run_id}_mission.csv"))
    robot = read_csv(os.path.join(LOG_DIR, f"{run_id}_robot.csv"))

    # Same rules as FactorySim: a violation is recorded when its kind differs from the last one.
    events = []
    for a in actions:
        if float(a["battery_end_percent"]) < reserve and a["location"] != "charger":
            events.append(("below_reserve", float(a["sim_start_s"])))
    for r in robot:
        if float(r["temperature_c"]) >= max_c:
            events.append(("overheated", float(r["ts"])))
        if float(r["battery_percent"]) <= 0.0:
            events.append(("battery_empty", float(r["ts"])))
    violations = []
    for kind, t in sorted(events, key=lambda e: e[1]):
        if not violations or violations[-1] != kind:
            violations.append(kind)

    delivered = sum(int(v) for v in (s["delivered_units"] or {}).values())
    energy = float(s["energy_drawn_wh"])
    nav = s.get("navigation") or {}
    score = (delivered * float(o["value_per_unit_delivered"]) - lost * float(o["cost_per_lost_unit"])
             - energy * float(o["cost_per_wh"])
             - len(violations) * float(o["cost_per_safety_violation"]))
    return {
        "score": round(score, 2), "delivered_units": delivered, "lost_units": round(lost, 2),
        "energy_wh": round(energy, 3), "wh_per_unit": round(energy / delivered, 4) if delivered else "",
        "distance_m": s["distance_m"],
        "charge_trips": sum(1 for a in actions if a["action"] == "CHARGE" and a["phase"] == "SUCCEEDED"),
        "min_battery_percent": round(min(float(r["battery_percent"]) for r in robot), 1) if robot else "",
        "violation_count": len(violations), "actions": s["actions"], "failed_actions": s["failed"],
        "fallback_decisions": (s.get("decisions") or {}).get("fallback", ""),
        "nav_failures": nav.get("goals_abandoned", 0), "shift_s": s["sim_duration_s"],
        "run_id": run_id,
    }


def sim_metrics(model, scenario, seed, shift_s):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from evaluate import build_policy
    from robofetch_ai.env.factory_sim import run_episode
    policy = build_policy(model, load_config("balanced", CONFIG_DIR))
    # finish_last_action: like the real executor, the action running at the end of the shift is
    # completed, so both sides cover the same work
    s, _ = run_episode(policy, scenario, seed, CONFIG_DIR, shift_duration_s=shift_s,
                       finish_last_action=True)
    keep = [f for f in FIELDS if f in s]
    out = {k: s[k] for k in keep}
    out["violation_count"] = len(s["violations"])
    out["wh_per_unit"] = s["wh_per_unit"] if s["wh_per_unit"] is not None else ""
    return out


def run_gazebo(model, scenario, seed, shift_s, log_path, timeout_s, planner=None):
    started = time.time()
    before = set(glob.glob(os.path.join(LOG_DIR, "run_*_mission_summary.yaml")))
    cmd = ["bash", os.path.join(WS, "scripts", "run.sh"), "--yes", "--no-build", "--headless",
           f"scenario:={scenario}", f"model:={model}", f"seed:={seed}", f"shift_s:={shift_s:.1f}",
           "web:=false"]
    if planner:                     # default: whatever config/run.yaml says
        cmd.append(f"planner:={planner}")
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, cwd=WS, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        summary = None
        while time.time() - started < timeout_s:
            new = set(glob.glob(os.path.join(LOG_DIR, "run_*_mission_summary.yaml"))) - before
            if new:
                summary = sorted(new)[-1]
                time.sleep(2.0)                     # let the file be fully written
                break
            if proc.poll() is not None:
                break
            time.sleep(5.0)
        # Stop only the run this script started: Ctrl+C to the launch, then the standard cleanup.
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
    subprocess.run(["bash", os.path.join(WS, "scripts", "stop.sh")], cwd=WS,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5.0)
    return summary, time.time() - started


def report(path):
    rows = read_csv(path)
    keys = sorted({(r["model"], r["scenario"]) for r in rows})
    metrics = ["score", "delivered_units", "lost_units", "energy_wh", "violation_count"]
    print(f"\nGazebo vs fast simulator (mean over seeds; gap = (gazebo - sim) / |sim|)")
    print(f"{'model':8s} {'scenario':18s} {'runs':>4s} " + " ".join(f"{m:>26.26s}" for m in metrics))
    for model, scenario in keys:
        sel = [r for r in rows if r["model"] == model and r["scenario"] == scenario]
        gz = [r for r in sel if r["source"] == "gazebo"]
        sm = [r for r in sel if r["source"] == "sim"]
        cells = []
        for m in metrics:
            g = statistics.fmean(float(r[m]) for r in gz) if gz else float("nan")
            f = statistics.fmean(float(r[m]) for r in sm) if sm else float("nan")
            gap = f"{100 * (g - f) / abs(f):+.0f}%" if f and abs(f) > 1e-9 else "   "
            cells.append(f"{g:8.2f} / {f:8.2f} {gap:>6s}")
        print(f"{model:8s} {scenario:18s} {len(gz):>4d} " + " ".join(f"{c:>26s}" for c in cells))
    print("(each cell: gazebo / fast sim  gap)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["ns", "ppo"])
    ap.add_argument("--scenarios", nargs="+",
                    default=["balanced", "low_battery_start", "section_breakdown"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[5000, 5001, 5002])
    ap.add_argument("--shift-min", type=float, default=20.0)
    ap.add_argument("--timeout-factor", type=float, default=1.5,
                    help="wall-clock limit per run = factor x shift + 35 min (startup plus a "
                         "last action that may be a full charge, which the executor finishes)")
    ap.add_argument("--planner", default=None,
                    help="global path planner for these runs (default: from config/run.yaml)")
    ap.add_argument("--skip-done", default=None, help="append to this CSV, skipping finished runs")
    ap.add_argument("--report-only", default=None)
    args = ap.parse_args()

    if args.report_only:
        report(args.report_only)
        return
    if simulation_running():
        print("a simulation is already running - stop it first (./scripts/stop.sh)")
        sys.exit(1)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    out = args.skip_done or os.path.join(RESULTS_DIR, time.strftime("gazebo_%Y%m%d_%H%M%S.csv"))
    done = set()
    if os.path.exists(out):
        done = {(r["model"], r["scenario"], r["seed"]) for r in read_csv(out) if r["source"] == "gazebo"}
    else:
        with open(out, "w", newline="") as fh:
            csv.DictWriter(fh, fieldnames=FIELDS).writeheader()

    cfg = load_config("balanced", CONFIG_DIR)
    shift_s = args.shift_min * 60.0
    runs = [(m, sc, sd) for sd in args.seeds for sc in args.scenarios for m in args.models]
    print(f"{len(runs)} Gazebo shifts of {args.shift_min:g} min -> {os.path.relpath(out, WS)}")
    for i, (model, scenario, seed) in enumerate(runs, 1):
        if (model, scenario, str(seed)) in done:
            print(f"[{i}/{len(runs)}] {model} {scenario} {seed}: already done")
            continue
        print(f"[{i}/{len(runs)}] {model} {scenario} seed {seed} ...", flush=True)
        tag = f"_{args.planner}" if args.planner else ""
        log_path = os.path.join(RESULTS_DIR, f"gazebo_{model}_{scenario}_{seed}{tag}.log")
        summary, wall = run_gazebo(model, scenario, seed, shift_s, log_path,
                                   args.timeout_factor * shift_s + 2100.0, args.planner)
        rows = []
        if summary:
            g = gazebo_metrics(summary, load_config(scenario, CONFIG_DIR))
            rows.append(dict(g, model=model, scenario=scenario, seed=seed, source="gazebo",
                             wall_s=round(wall)))
            print(f"    gazebo: score {g['score']}, delivered {g['delivered_units']}, lost "
                  f"{g['lost_units']}, {g['energy_wh']} Wh, violations {g['violation_count']}, "
                  f"failed actions {g['failed_actions']}, {wall / 60:.1f} min wall", flush=True)
        else:
            print(f"    gazebo: NO SUMMARY within the time limit (see {os.path.relpath(log_path, WS)})")
        sim = sim_metrics(model, scenario, seed, shift_s)
        rows.append(dict(sim, model=model, scenario=scenario, seed=seed, source="sim"))
        print(f"    sim   : score {sim['score']}, delivered {sim['delivered_units']}, lost "
              f"{sim['lost_units']}, {sim['energy_wh']} Wh, violations {sim['violation_count']}",
              flush=True)
        with open(out, "a", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
            for r in rows:
                writer.writerow(r)
    report(out)


if __name__ == "__main__":
    main()
