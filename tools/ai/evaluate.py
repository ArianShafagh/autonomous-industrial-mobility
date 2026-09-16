#!/usr/bin/env python3
"""Run decision policies over scenarios and seeds in the fast simulator, and report.

    venv/bin/python tools/ai/evaluate.py                                  # rule policy, all scenarios
    venv/bin/python tools/ai/evaluate.py --policies rule --seeds 30
    venv/bin/python tools/ai/evaluate.py --scenarios balanced high_demand --trace

Writes one row per episode to tools/ai/results/episodes_<timestamp>.csv and prints the mean of
every metric per policy and scenario, with a 95 % bootstrap confidence interval on the score.
Policies are compared on the SAME seeds, so the comparison is paired.
"""
import argparse
import csv
import os
import statistics
import sys
import time

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

from robofetch_ai.env.factory_sim import run_episode  # noqa: E402
from robofetch_ai.policies.rule_based import RuleBasedPolicy  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")
RESULTS_DIR = os.path.join(WS, "tools", "ai", "results")

POLICIES = {
    "rule": RuleBasedPolicy,
    # "ns"  : NeuroSymbolicPolicy   (WP5)
    # "ppo" : PPOPolicy             (WP6)
}

METRICS = ["score", "delivered_units", "lost_units", "energy_wh", "wh_per_unit", "distance_m",
           "charge_trips", "min_battery_percent", "actions"]


def bootstrap_ci(values, samples=2000, alpha=0.05, seed=0):
    import random
    rng = random.Random(seed)
    if len(values) < 2:
        return (float("nan"), float("nan"))
    means = sorted(statistics.fmean(rng.choices(values, k=len(values))) for _ in range(samples))
    lo = means[int(alpha / 2 * samples)]
    hi = means[int((1 - alpha / 2) * samples) - 1]
    return lo, hi


def scenarios_available():
    return sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policies", nargs="+", default=["rule"], choices=sorted(POLICIES))
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--seeds", type=int, default=20, help="number of seeds per scenario")
    ap.add_argument("--shift-s", type=float, default=None, help="override shift length")
    ap.add_argument("--trace", action="store_true", help="print the decisions of the first episode")
    args = ap.parse_args()

    scenarios = args.scenarios or scenarios_available()
    os.makedirs(RESULTS_DIR, exist_ok=True)
    out_path = os.path.join(RESULTS_DIR, time.strftime("episodes_%Y%m%d_%H%M%S.csv"))
    rows = []
    t0 = time.time()

    for policy_name in args.policies:
        policy = POLICIES[policy_name]()
        for scenario in scenarios:
            for seed in range(args.seeds):
                policy.reset()
                summary, decisions = run_episode(
                    policy, scenario, seed, CONFIG_DIR, shift_duration_s=args.shift_s,
                    trace=args.trace and seed == 0)
                summary["policy"] = policy_name
                summary["violation_count"] = len(summary["violations"])
                rows.append(summary)
                if args.trace and seed == 0:
                    print(f"\n--- {policy_name} / {scenario} / seed 0: first decisions")
                    for d in decisions[:12]:
                        print(f"  t={d['time_s']:7.1f} {d['action']:<12s} {d['why']}")

    keys = sorted({k for r in rows for k in r})
    with open(out_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (",".join(v) if isinstance(v, list) else v) for k, v in r.items()})

    print(f"\n{len(rows)} episodes in {time.time() - t0:.1f} s -> {os.path.relpath(out_path, WS)}")
    header = f"{'policy':8s} {'scenario':18s}" + "".join(f"{m:>16s}" for m in METRICS)
    print("\n" + header)
    for policy_name in args.policies:
        for scenario in scenarios:
            sel = [r for r in rows if r["policy"] == policy_name and r["scenario"] == scenario]
            if not sel:
                continue
            line = f"{policy_name:8s} {scenario:18s}"
            for m in METRICS:
                vals = [r[m] for r in sel if r[m] is not None]
                line += f"{statistics.fmean(vals):16.2f}" if vals else f"{'-':>16s}"
            lo, hi = bootstrap_ci([r["score"] for r in sel])
            print(line + f"   score 95% CI [{lo:.1f}, {hi:.1f}]")
            viol = sum(r["violation_count"] for r in sel)
            if viol:
                print(f"{'':27s}   safety violations in {viol} of {len(sel)} episodes")


if __name__ == "__main__":
    main()
