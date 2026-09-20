#!/usr/bin/env python3
"""Thesis figures from the WP9 results.

    venv/bin/python tools/ai/plot_results.py --episodes tools/ai/results/episodes_<ts>.csv \\
        [--gazebo tools/ai/results/gazebo_<ts>.csv]

Writes PNGs next to the input CSV (tools/ai/results/fig_*.png):
    fig_score_by_scenario   score per policy, one panel per scenario, 95 % bootstrap CI
    fig_energy_throughput   delivered units vs Wh per delivered unit, one point per policy
    fig_violations          share of shifts with a safety violation, per policy
    fig_gazebo_vs_sim       Gazebo score vs fast-simulator score on the same seed (y = x is perfect)
"""
import argparse
import csv
import os
import random
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REFERENCE = "ns"
HIGHLIGHT = "#2a78d6"      # the thesis model
OTHER = "#9a9893"          # every comparison policy (identity is on the axis, not in colour)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]    # validated all-pairs slots, max 3 series
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
# The ns variants sit close together in this chart; fixed offsets keep their labels apart.
LABEL_OFFSETS = {"ns": (0, -18, "center"), "ns_unbounded": (-8, -10, "right"),
                 "ns_estimate_only": (-8, 12, "right"), "ns_symbolic_only": (8, 18, "left"),
                 "ns_neural_only": (8, -12, "left")}
ORDER = ["ns", "ns_unbounded", "ns_estimate_only", "ns_symbolic_only", "ns_neural_only", "rule",
         "ppo", "ppo_unmasked", "ppo_wp6"]

plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "figure.dpi": 150, "savefig.bbox": "tight",
})


def read(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def num(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def ci(values, samples=2000, seed=0):
    rng = random.Random(seed)
    means = sorted(statistics.fmean(rng.choices(values, k=len(values))) for _ in range(samples))
    return means[int(0.025 * samples)], means[int(0.975 * samples) - 1]


def policies_in(rows):
    present = {r["policy"] for r in rows}
    return [p for p in ORDER if p in present] + sorted(present - set(ORDER))


def score_by_scenario(rows, seen, out):
    scenarios = sorted({r["scenario"] for r in rows}, key=lambda s: (s not in seen, s))
    policies = policies_in(rows)
    cols = 4
    nrows = (len(scenarios) + cols - 1) // cols
    fig, axes = plt.subplots(nrows, cols, figsize=(11, 2.1 * nrows + 0.4), sharey=True,
                             squeeze=False)
    for ax, scenario in zip(axes.flat, scenarios):
        for y, policy in enumerate(policies):
            vals = [num(r["score"]) for r in rows if r["scenario"] == scenario and r["policy"] == policy]
            if not vals:
                continue
            mean = statistics.fmean(vals)
            lo, hi = ci(vals)
            unsafe = sum(1 for r in rows if r["scenario"] == scenario and r["policy"] == policy
                         and num(r["violation_count"]))
            color = HIGHLIGHT if policy == REFERENCE else OTHER
            # a score reached with safety violations is not a real result: hatch it and say so
            ax.barh(y, mean, height=0.62, color="white" if unsafe else color,
                    edgecolor=color if unsafe else "white", hatch="////" if unsafe else None,
                    linewidth=1)
            ax.plot([lo, hi], [y, y], color=INK, linewidth=1)
            if unsafe:
                ax.annotate(f"unsafe {unsafe}/{len(vals)}", (max(hi, 0), y), xytext=(3, 0),
                            textcoords="offset points", va="center", fontsize=6.5, color=MUTED)
        ax.axvline(0, color=MUTED, linewidth=0.8)
        ax.set_title(f"{scenario}  ({'seen' if scenario in seen else 'unseen'})", fontsize=9,
                     color=INK if scenario in seen else MUTED, loc="left")
        ax.grid(axis="y", visible=False)
        ax.margins(x=0.22)
    for ax in axes.flat[len(scenarios):]:
        ax.set_visible(False)
    for ax in axes[:, 0]:
        ax.set_yticks(range(len(policies)), policies)
    axes[0, 0].invert_yaxis()           # shared y: once inverts every panel
    fig.suptitle("Shift score per policy and scenario (mean, 95 % CI; higher is better; "
                 "hatched = shifts with safety violations)",
                 x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def energy_throughput(rows, out):
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for policy in policies_in(rows):
        sel = [r for r in rows if r["policy"] == policy]
        delivered = statistics.fmean(num(r["delivered_units"]) for r in sel)
        per_unit = [num(r["wh_per_unit"]) for r in sel if num(r["wh_per_unit"]) is not None]
        if not per_unit:
            continue
        x, y = delivered, statistics.fmean(per_unit)
        color = HIGHLIGHT if policy == REFERENCE else OTHER
        ax.scatter([x], [y], s=46, color=color, edgecolor="white", linewidth=1.5, zorder=3)
        dx, dy, ha = LABEL_OFFSETS.get(policy, (6, 4, "left"))
        ax.annotate(policy, (x, y), xytext=(dx, dy), textcoords="offset points", fontsize=8,
                    color=INK, ha=ha, va="center",
                    arrowprops={"arrowstyle": "-", "color": MUTED, "linewidth": 0.6,
                                "shrinkA": 0, "shrinkB": 4} if abs(dy) > 8 else None)
    ax.set_xlabel("delivered units per shift (mean over all scenarios and seeds)")
    ax.set_ylabel("Wh per delivered unit")
    ax.set_title("Throughput vs energy efficiency (right and low is better)", loc="left",
                 fontsize=11, color=INK)
    fig.savefig(out)
    plt.close(fig)


def violations(rows, out):
    policies = policies_in(rows)
    shares = []
    for policy in policies:
        sel = [r for r in rows if r["policy"] == policy]
        shares.append(100.0 * sum(1 for r in sel if num(r["violation_count"])) / len(sel))
    fig, ax = plt.subplots(figsize=(6.4, 0.34 * len(policies) + 1.0))
    colors = [HIGHLIGHT if p == REFERENCE else OTHER for p in policies]
    ax.barh(range(len(policies)), shares, height=0.62, color=colors)
    for y, share in enumerate(shares):
        ax.annotate(f"{share:.1f} %", (share, y), xytext=(4, 0), textcoords="offset points",
                    va="center", fontsize=8, color=INK)
    ax.set_yticks(range(len(policies)), policies)
    ax.invert_yaxis()
    ax.grid(axis="y", visible=False)
    ax.set_xlim(0, max(10.0, max(shares) * 1.2))
    ax.set_xlabel("shifts with at least one safety violation (%)")
    ax.set_title("Safety: battery below reserve, overheating or empty battery", loc="left",
                 fontsize=11, color=INK)
    fig.savefig(out)
    plt.close(fig)


def gazebo_vs_sim(rows, out):
    runs = {}
    for r in rows:
        runs.setdefault((r["model"], r["scenario"], r["seed"]), {})[r["source"]] = num(r["score"])
    models = sorted({k[0] for k in runs}, key=lambda m: ORDER.index(m) if m in ORDER else 99)[:3]
    fig, ax = plt.subplots(figsize=(5.2, 5.0))
    points = []
    for i, model in enumerate(models):
        pairs = [(v["sim"], v["gazebo"]) for k, v in runs.items()
                 if k[0] == model and "sim" in v and "gazebo" in v]
        if not pairs:
            continue
        points += pairs
        ax.scatter([p[0] for p in pairs], [p[1] for p in pairs], s=46, color=SERIES[i],
                   edgecolor="white", linewidth=1.5, label=model, zorder=3)
    if points:
        lo = min(min(p) for p in points) - 2
        hi = max(max(p) for p in points) + 2
        ax.plot([lo, hi], [lo, hi], color=MUTED, linewidth=1, linestyle="--", zorder=2)
        ax.annotate("y = x", (hi, hi), xytext=(-30, -12), textcoords="offset points",
                    fontsize=8, color=MUTED)
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
    ax.set_xlabel("fast simulator score (same policy, scenario, seed)")
    ax.set_ylabel("Gazebo score")
    ax.legend(frameon=False, loc="upper left")
    ax.set_title("Gazebo vs fast simulator, one point per shift", loc="left", fontsize=11, color=INK)
    fig.savefig(out)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", required=True)
    ap.add_argument("--gazebo", default=None)
    args = ap.parse_args()
    out_dir = os.path.dirname(os.path.abspath(args.episodes))

    rows = read(args.episodes)
    seen = {r["scenario"] for r in rows if r.get("split") == "seen"}
    main_rows = [r for r in rows if r["policy"] != "ppo_wp6"]
    made = []
    for name, fn, extra in [("fig_score_by_scenario.png", score_by_scenario, (seen,)),
                            ("fig_energy_throughput.png", energy_throughput, ()),
                            ("fig_violations.png", violations, ())]:
        path = os.path.join(out_dir, name)
        fn(main_rows, *extra, path)
        made.append(path)
    if args.gazebo:
        path = os.path.join(out_dir, "fig_gazebo_vs_sim.png")
        gazebo_vs_sim(read(args.gazebo), path)
        made.append(path)
    for path in made:
        print(path)


if __name__ == "__main__":
    main()
