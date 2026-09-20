#!/usr/bin/env python3
"""Run decision policies over scenarios and seeds in the fast simulator, and report.

    venv/bin/python tools/ai/evaluate.py                                  # rule + ns, all scenarios
    venv/bin/python tools/ai/evaluate.py --policies all --workers 10      # WP9 tier 1
    venv/bin/python tools/ai/evaluate.py --scenarios balanced high_demand --trace

Seeds start at `mission.evaluation.test_seed_start` (params.yaml), which no training or model
selection ever used, and every policy runs the SAME seeds, so comparisons are paired.

Scenarios are tagged `seen` (in `training_scenarios`, the six both learned models trained on) or
`unseen` (written after training: generalisation).

Writes to tools/ai/results/:
    episodes_<ts>.csv   one row per episode
    summary_<ts>.md     per scenario: mean and 95 % bootstrap CI of every metric; paired Wilcoxon
                        signed-rank test of every policy's score against --reference; the same
                        pooled over seen and unseen scenarios
"""
import argparse
import csv
import os
import random
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

from robofetch_factory.factory_model import load_config  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")
RESULTS_DIR = os.path.join(WS, "tools", "ai", "results")
CHECKPOINTS = os.path.join(WS, "tools", "ai", "checkpoints")

POLICIES = ["rule", "ns", "ns_unbounded", "ns_estimate_only", "ns_neural_only", "ns_symbolic_only",
            "ppo", "ppo_unmasked", "ppo_wp6"]

METRICS = ["score", "delivered_units", "lost_units", "energy_wh", "wh_per_unit", "distance_m",
           "charge_trips", "min_battery_percent", "violation_count", "actions", "decision_ms"]
SHOWN = ["score", "delivered_units", "lost_units", "energy_wh", "wh_per_unit", "charge_trips",
         "min_battery_percent", "violation_count", "decision_ms"]


def build_policy(name, cfg):
    from robofetch_ai.policies.neurosymbolic import NeuroSymbolicPolicy
    from robofetch_ai.policies.rl_ppo import DEFAULT_MODEL, PPOPolicy
    from robofetch_ai.policies.rule_based import RuleBasedPolicy
    if name == "rule":
        return RuleBasedPolicy()
    if name == "ns_unbounded":      # the WP5 model: the network's correction is not bounded
        return NeuroSymbolicPolicy(cfg, mode="full", max_correction=None)
    if name == "ns_estimate_only":  # rules + symbolic estimate, network correction switched off
        return NeuroSymbolicPolicy(cfg, mode="full", max_correction=0.0)
    if name.startswith("ns"):
        mode = {"ns": "full", "ns_neural_only": "neural", "ns_symbolic_only": "symbolic"}[name]
        return NeuroSymbolicPolicy(cfg, mode=mode)
    if name == "ppo":
        return PPOPolicy(cfg)
    if name == "ppo_unmasked":
        return PPOPolicy(cfg, model_path=DEFAULT_MODEL.replace(".zip", "_unmasked.zip"),
                         use_masks=False)
    if name == "ppo_wp6":       # the WP6 model, trained on only 4 of the 6 training scenarios
        return PPOPolicy(cfg, model_path=os.path.join(CHECKPOINTS, "ppo_policy_wp6.zip"))
    raise ValueError(f"unknown policy '{name}'")


def run_block(job):
    """All seeds of one policy on one scenario (one worker process)."""
    policy_name, scenario, seeds, shift_s, trace = job
    import torch
    torch.set_num_threads(1)
    from robofetch_ai.env.factory_sim import run_episode
    cfg = load_config("balanced", CONFIG_DIR)
    policy = build_policy(policy_name, cfg)
    rows, lines = [], []
    for seed in seeds:
        policy.reset()
        summary, decisions = run_episode(policy, scenario, seed, CONFIG_DIR,
                                         shift_duration_s=shift_s, trace=trace and seed == seeds[0])
        summary["policy"] = policy_name
        summary["violation_count"] = len(summary["violations"])
        rows.append(summary)
        if trace and seed == seeds[0]:
            lines.append(f"\n--- {policy_name} / {scenario} / seed {seed}: first decisions")
            lines += [f"  t={d['time_s']:7.1f} {d['action']:<12s} {d['why']}" for d in decisions[:12]]
    return rows, lines


def bootstrap_ci(values, samples=2000, alpha=0.05, seed=0):
    rng = random.Random(seed)
    if len(values) < 2:
        return (float("nan"), float("nan"))
    means = sorted(statistics.fmean(rng.choices(values, k=len(values))) for _ in range(samples))
    return means[int(alpha / 2 * samples)], means[int((1 - alpha / 2) * samples) - 1]


def wilcoxon(a, b):
    """Paired two-sided Wilcoxon signed-rank p-value; 1.0 when the two are identical."""
    from scipy.stats import wilcoxon as scipy_wilcoxon
    diffs = [x - y for x, y in zip(a, b)]
    if all(abs(d) < 1e-9 for d in diffs):
        return 1.0
    return float(scipy_wilcoxon(a, b, zero_method="zsplit").pvalue)


def scenarios_available():
    return sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))


def fmt_p(p):
    return "<0.001" if p < 0.001 else f"{p:.3f}"


def write_summary(path, rows, policies, scenarios, seen, reference, args):
    md = [f"# Fast-simulator evaluation ({time.strftime('%Y-%m-%d %H:%M')})", "",
          f"{args.seeds} seeds per policy and scenario, seeds {args.seed_start}..."
          f"{args.seed_start + args.seeds - 1} (never used in training), shift "
          f"{args.shift_s or 'from params.yaml'} s. Values are means; [low, high] is the 95 % "
          f"bootstrap CI of the score. p = paired Wilcoxon signed-rank test of the score against "
          f"`{reference}` on the same seeds.", "",
          "score = delivered units - lost production - 0.5 x Wh - 20 x safety violations", ""]

    def table(title, selector):
        md.extend([f"## {title}", "",
                   "| policy | score [95 % CI] | delivered | lost | Wh | Wh/unit | charges | "
                   "min batt % | episodes w/ violation | ms/decision | Δ score vs ref | p |",
                   "|---|---|---|---|---|---|---|---|---|---|---|---|"])
        ref = {(r["scenario"], r["seed"]): r["score"] for r in rows
               if r["policy"] == reference and selector(r)}
        for policy in policies:
            sel = [r for r in rows if r["policy"] == policy and selector(r)]
            if not sel:
                continue

            def mean(key):
                vals = [r[key] for r in sel if r[key] is not None]
                return statistics.fmean(vals) if vals else float("nan")
            lo, hi = bootstrap_ci([r["score"] for r in sel])
            pairs = [(r["score"], ref[(r["scenario"], r["seed"])]) for r in sel
                     if (r["scenario"], r["seed"]) in ref]
            if policy == reference or not pairs:
                delta, p = "—", "—"
            else:
                delta = f"{statistics.fmean(a - b for a, b in pairs):+.2f}"
                p = fmt_p(wilcoxon([a for a, _ in pairs], [b for _, b in pairs]))
            violated = sum(1 for r in sel if r["violation_count"])
            name = f"**{policy}**" if policy == reference else policy
            md.append(f"| {name} | {mean('score'):.2f} [{lo:.1f}, {hi:.1f}] | "
                      f"{mean('delivered_units'):.1f} | {mean('lost_units'):.2f} | "
                      f"{mean('energy_wh'):.2f} | {mean('wh_per_unit'):.3f} | "
                      f"{mean('charge_trips'):.1f} | {mean('min_battery_percent'):.1f} | "
                      f"{violated}/{len(sel)} | {mean('decision_ms'):.2f} | {delta} | {p} |")
        md.append("")

    table("All scenarios pooled", lambda r: True)
    table("Seen scenarios (trained on)", lambda r: r["scenario"] in seen)
    table("Unseen scenarios (generalisation)", lambda r: r["scenario"] not in seen)
    for scenario in scenarios:
        table(f"{scenario} ({'seen' if scenario in seen else 'unseen'})",
              lambda r, s=scenario: r["scenario"] == s)
    with open(path, "w") as fh:
        fh.write("\n".join(md))


def main():
    cfg = load_config("balanced", CONFIG_DIR)
    ev = cfg["mission"]["evaluation"]
    ap = argparse.ArgumentParser()
    ap.add_argument("--policies", nargs="+", default=["rule", "ns"], choices=POLICIES + ["all"])
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--seeds", type=int, default=int(ev["seeds"]))
    ap.add_argument("--seed-start", type=int, default=int(ev["test_seed_start"]))
    ap.add_argument("--shift-s", type=float, default=None, help="override shift length")
    ap.add_argument("--reference", default="ns", help="policy the others are tested against")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--trace", action="store_true", help="print the decisions of the first episode")
    args = ap.parse_args()

    policies = POLICIES if "all" in args.policies else args.policies
    scenarios = args.scenarios or scenarios_available()
    seen = set(ev["training_scenarios"])
    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    os.makedirs(RESULTS_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(RESULTS_DIR, f"episodes_{stamp}.csv")
    out_md = os.path.join(RESULTS_DIR, f"summary_{stamp}.md")

    jobs = [(p, s, seeds, args.shift_s, args.trace) for p in policies for s in scenarios]
    print(f"{len(policies)} policies x {len(scenarios)} scenarios x {len(seeds)} seeds = "
          f"{len(jobs) * len(seeds)} episodes on {args.workers} workers", flush=True)
    t0 = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i, (block, lines) in enumerate(pool.map(run_block, jobs), 1):
            rows += block
            for line in lines:
                print(line)
            if i % max(1, len(jobs) // 10) == 0:
                print(f"  {i}/{len(jobs)} blocks done, {time.time() - t0:.0f} s", flush=True)
    for r in rows:
        r["split"] = "seen" if r["scenario"] in seen else "unseen"

    keys = sorted({k for r in rows for k in r})
    with open(out_csv, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (",".join(v) if isinstance(v, list) else v) for k, v in r.items()})

    reference = args.reference if args.reference in policies else policies[0]
    write_summary(out_md, rows, policies, scenarios, seen, reference, args)

    print(f"\n{len(rows)} episodes in {time.time() - t0:.1f} s")
    header = f"{'policy':16s} {'scenario':18s}" + "".join(f"{m:>12.12s}" for m in SHOWN)
    print("\n" + header)
    for policy_name in policies:
        for scenario in scenarios:
            sel = [r for r in rows if r["policy"] == policy_name and r["scenario"] == scenario]
            if not sel:
                continue
            line = f"{policy_name:16s} {scenario:18s}"
            for m in SHOWN:
                vals = [r[m] for r in sel if r[m] is not None]
                line += f"{statistics.fmean(vals):12.2f}" if vals else f"{'-':>12s}"
            print(line)
    print(f"\nepisodes -> {os.path.relpath(out_csv, WS)}")
    print(f"summary  -> {os.path.relpath(out_md, WS)}")


if __name__ == "__main__":
    main()
