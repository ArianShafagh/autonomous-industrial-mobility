#!/usr/bin/env python3
"""What do the numbers in params.yaml imply? Run this after every parameter change.

    venv/bin/python scripts/param_report.py                         # balanced scenario
    venv/bin/python scripts/param_report.py --scenario high_demand
    venv/bin/python scripts/param_report.py --set robot.battery.capacity_wh=30 \\
                                            --set factory.sections.B.rate_per_hour=45
    venv/bin/python scripts/param_report.py --all                   # one summary line per scenario

It prints the robot's range/runtime/charging figures, the cost of one full-load trip to each
section, how much of the robot's time the factory demands (utilisation, including charging), how
fast each line blocks if the robot never comes, and warnings for combinations that cannot work.
Everything uses the SAME model code as the live system and the fast simulator.
"""
import argparse
import os
import statistics
import sys

import yaml

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(WS, "src", "robofetch_core"))
sys.path.insert(0, os.path.join(WS, "src", "robofetch_factory"))

from robofetch_core.robot_model import (RobotCondition, RobotParams, battery_percent_for,  # noqa: E402
                                        charge_time_s, idle_energy_wh, net_charge_power_w,
                                        simulate_route, travel_time_s, trip_energy_wh)
from robofetch_factory.factory_model import build_sections, load_config  # noqa: E402
from robofetch_factory.layout import load_path_matrix  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")


def parse_set(items):
    """--set a.b.c=value  ->  nested override dict (value parsed as YAML: numbers stay numbers)."""
    out = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"--set expects key=value, got '{item}'")
        key, value = item.split("=", 1)
        node = out
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = yaml.safe_load(value)
    return out


def analyse(cfg, matrix, seeds=5):
    p = RobotParams.from_config(cfg)
    ts = float(cfg["time"]["time_scale"])
    shift = float(cfg["time"]["shift_duration_s"])
    sections = cfg["factory"]["sections"]
    defaults = cfg["factory"]["section_defaults"]
    dwell = p.load_time_s + p.unload_time_s
    r = {"p": p, "warnings": [], "trips": {}, "sections": {}}

    # ------------------------------------------------------------------ robot
    cond = p.initial_condition_percent            # a worn drive draws more on every trip
    per_km = trip_energy_wh(p, 1000.0, 0.0, condition_percent=cond)
    drive_power = per_km * p.speed_m_s * 3.6
    r["range_empty_m"] = p.capacity_wh / (per_km / 1000.0)
    r["range_full_m"] = p.capacity_wh / (
        trip_energy_wh(p, 1000.0, p.max_payload_kg, condition_percent=cond) / 1000.0)
    r["runtime_driving_h"] = p.capacity_wh / drive_power
    r["runtime_idle_h"] = p.capacity_wh / p.idle_power_w
    r["charge_reserve_to_full_min"] = charge_time_s(p, p.reserve_percent) / 60.0
    r["per_km_wh"] = per_km
    r["drive_power_w"] = drive_power
    if net_charge_power_w(p) <= 0:
        r["warnings"].append("charge_power_w <= idle_power_w: the robot can never charge")

    # --------------------------------------------------------- trips and demand
    busy_s_per_h = 0.0
    energy_wh_per_h = 0.0
    kg_per_h_total = 0.0
    for sid, sec in sections.items():
        rate = sec["rate_per_hour"] * ts                    # units per SIM hour
        full_kg = sec["buffer_capacity"] * sec["unit_mass_kg"]
        load_kg = min(full_kg, p.max_payload_kg)
        if full_kg > p.max_payload_kg:
            r["warnings"].append(f"{sid}: full buffer {full_kg:.1f} kg exceeds payload "
                                 f"{p.max_payload_kg:.1f} kg - one trip cannot unblock it")
        d = matrix["delivery"][sid]
        drive_s = 2 * travel_time_s(p, d)
        trip_s = drive_s + dwell
        trip_wh = (trip_energy_wh(p, d, 0.0, condition_percent=cond)
                   + trip_energy_wh(p, d, load_kg, condition_percent=cond) + idle_energy_wh(p, dwell))
        kg_per_h = rate * sec["unit_mass_kg"]
        trips_per_h = kg_per_h / load_kg if load_kg > 0 else 0.0
        busy_s_per_h += trips_per_h * trip_s
        energy_wh_per_h += trips_per_h * (trip_wh - idle_energy_wh(p, trip_s))
        kg_per_h_total += kg_per_h

        # worst single mission: charger -> section -> delivery -> charger, must keep the reserve
        worst, peak = simulate_route(RobotCondition(p), [(matrix["charger"][sid], 0.0),
                                                        (d, load_kg), (matrix["delivery"]["charger"], 0.0)],
                                     dwell_s=dwell)
        if worst.battery_percent < p.reserve_percent:
            r["warnings"].append(f"{sid}: a single mission from a FULL battery ends at "
                                 f"{worst.battery_percent:.0f} % (< reserve)")
        if peak >= p.max_c:
            r["warnings"].append(f"{sid}: a full-load trip overheats the motors ({peak:.0f} C)")
        fill_s = sec["buffer_capacity"] / (rate / 3600.0) if rate > 0 else float("inf")
        r["trips"][sid] = dict(distance_m=d, load_kg=load_kg, trip_s=trip_s, trip_wh=trip_wh,
                               trip_pct=battery_percent_for(p, trip_wh), trips_per_h=trips_per_h,
                               kg_per_h=kg_per_h, fill_from_empty_s=fill_s, peak_c=peak)

    energy_wh_per_h += idle_energy_wh(p, 3600.0)            # electronics run all hour
    charge_s_per_h = energy_wh_per_h / max(1e-9, net_charge_power_w(p)) * 3600.0
    r["kg_per_h"] = kg_per_h_total
    r["busy_fraction"] = busy_s_per_h / 3600.0
    r["charge_fraction"] = charge_s_per_h / 3600.0
    r["utilisation"] = r["busy_fraction"] + r["charge_fraction"]
    r["energy_wh_per_h"] = energy_wh_per_h
    if r["utilisation"] > 1.0:
        r["warnings"].append(f"robot utilisation {r['utilisation']:.2f} > 1: the robot cannot keep "
                             "up even with perfect decisions - production will be lost")

    # --------------------------------------------------- no-robot shift simulation
    first_block = {sid: [] for sid in sections}
    lost = {sid: [] for sid in sections}
    faults = {sid: [] for sid in sections}
    for seed in range(seeds):
        secs = build_sections(cfg, seed)
        seen = {sid: None for sid in secs}
        for t in range(int(shift)):
            for sid, s in secs.items():
                s.step(1.0)
                if seen[sid] is None and s.status == "BLOCKED":
                    seen[sid] = t + 1
        for sid, s in secs.items():
            first_block[sid].append(seen[sid] if seen[sid] is not None else float("inf"))
            lost[sid].append(s.lost_units)
            faults[sid].append(s.faults_total)
    for sid in sections:
        fb = first_block[sid]
        r["sections"][sid] = dict(first_block_s=statistics.mean(fb) if all(x != float("inf") for x in fb)
                                  else float("inf"), lost_units=statistics.mean(lost[sid]),
                                  faults=statistics.mean(faults[sid]))
    r["expected_faults_per_shift"] = defaults["faults_per_hour"] * shift * ts / 3600.0
    return r


def fmt_s(seconds):
    if seconds == float("inf"):
        return "never"
    return f"{seconds / 60:5.1f} min"


def print_report(cfg, r):
    p = r["p"]
    print(f"\n=== scenario: {cfg['name']} — {cfg['description']}")
    print(f"time_scale {cfg['time']['time_scale']:g}, shift {cfg['time']['shift_duration_s']:.0f} s, "
          f"robot starts at {p.initial_battery_percent:g} % battery, condition {p.initial_condition_percent:g} %")
    print("\nROBOT")
    print(f"  battery {p.capacity_wh:g} Wh, reserve {p.reserve_percent:g} %, idle {p.idle_power_w:g} W, "
          f"driving {r['drive_power_w']:.1f} W total at {p.speed_m_s:g} m/s")
    print(f"  energy per km (empty, incl. idle) {r['per_km_wh']:.1f} Wh")
    print(f"  range: empty {r['range_empty_m']:.0f} m, with {p.max_payload_kg:g} kg {r['range_full_m']:.0f} m")
    print(f"  runtime: driving non-stop {r['runtime_driving_h']:.2f} h, standing idle {r['runtime_idle_h']:.2f} h")
    print(f"  charging {p.charge_power_w:g} W (net {net_charge_power_w(p):g} W): "
          f"reserve -> full in {r['charge_reserve_to_full_min']:.0f} min")
    print("\nFULL-LOAD TRIP  delivery -> section -> delivery  (incl. load+unload dwell)")
    print("  sec  dist(m)  load(kg)  time(s)  energy(Wh)  battery  peak motor  trips/h needed  kg/h")
    for sid, t in r["trips"].items():
        print(f"   {sid}   {t['distance_m']:6.2f}   {t['load_kg']:6.2f}   {t['trip_s']:6.0f}   "
              f"{t['trip_wh']:8.2f}   {t['trip_pct']:5.1f} %   {t['peak_c']:5.1f} C     "
              f"{t['trips_per_h']:6.2f}       {t['kg_per_h']:5.1f}")
    print("\nDEMAND vs ROBOT")
    print(f"  factory output {r['kg_per_h']:.1f} kg per sim hour")
    print(f"  robot busy driving/handling {100 * r['busy_fraction']:5.1f} % of the time")
    print(f"  energy needed {r['energy_wh_per_h']:.1f} Wh/h -> charging {100 * r['charge_fraction']:5.1f} % of the time")
    print(f"  UTILISATION (busy + charging) = {r['utilisation']:.2f}   "
          "(<0.5 easy, 0.6-0.9 decisions matter, >1 impossible to keep up)")
    print("\nIF THE ROBOT NEVER CAME (shift simulation, 5 seeds)")
    print("  sec  buffer fills from empty   first BLOCKED   lost units/shift   faults/shift")
    for sid, s in r["sections"].items():
        print(f"   {sid}      {fmt_s(r['trips'][sid]['fill_from_empty_s'])}           {fmt_s(s['first_block_s'])}"
              f"        {s['lost_units']:7.1f}           {s['faults']:.2f}")
    if r["warnings"]:
        print("\nWARNINGS")
        for w in r["warnings"]:
            print("  ! " + w)
    else:
        print("\nno warnings")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="balanced")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override a parameter, e.g. robot.battery.capacity_wh=30 (repeatable)")
    ap.add_argument("--all", action="store_true", help="one summary line per scenario")
    args = ap.parse_args()

    overrides = parse_set(args.set)
    matrix = load_path_matrix(CONFIG_DIR)
    if args.all:
        names = sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))
        print("scenario            util  busy%  charge%  kg/h   first-block A/B/C (min)   warnings")
        for name in names:
            cfg = load_config(name, CONFIG_DIR, overrides)
            r = analyse(cfg, matrix)
            fb = "/".join(fmt_s(r["sections"][s]["first_block_s"]).strip().replace(" min", "")
                          for s in r["sections"])
            print(f"{name:18s}  {r['utilisation']:4.2f}  {100 * r['busy_fraction']:5.1f}  "
                  f"{100 * r['charge_fraction']:6.1f}  {r['kg_per_h']:5.1f}   {fb:24s}  {len(r['warnings'])}")
        return
    cfg = load_config(args.scenario, CONFIG_DIR, overrides)
    print_report(cfg, analyse(cfg, matrix))


if __name__ == "__main__":
    main()
