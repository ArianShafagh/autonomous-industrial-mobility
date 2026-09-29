#!/usr/bin/env python3
"""Re-derive the numbers the thesis states from the raw data they came from.

The thesis asserts numbers; this script recomputes each one from the episode CSVs and
the configuration files, using the same estimators as `tools/ai/evaluate.py`
(2000-resample bootstrap seeded at 0, paired Wilcoxon signed-rank), and reports every
cell that disagrees.

    python tools/thesis/check_claims.py --thesis /path/to/thesis

Exit code 0 = every claim matches its data, 1 = at least one does not.
"""
import argparse
import csv
import pathlib
import random
import re
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
COLS = ["ns", "ns_symbolic_only", "ppo", "rule"]          # columns of tab:per-scenario
WORDS = {"four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
         "ten": 10, "eleven": 11, "twelve": 12}


def bootstrap_ci(values, samples=2000, alpha=0.05, seed=0):
    """Identical to evaluate.bootstrap_ci, so the CI must reproduce exactly."""
    rng = random.Random(seed)
    if not values:
        return 0.0, 0.0
    means = sorted(statistics.fmean(rng.choices(values, k=len(values))) for _ in range(samples))
    return means[int(alpha / 2 * samples)], means[int((1 - alpha / 2) * samples)]


def wilcoxon(a, b):
    from scipy.stats import wilcoxon as scipy_wilcoxon
    if all(x == y for x, y in zip(a, b)):
        return 1.0
    return float(scipy_wilcoxon(a, b, zero_method="zsplit").pvalue)


def num(cell):
    """First number in a LaTeX cell, honouring \\num{}, \\textbf{}, $-$ and en-dashes."""
    cell = cell.replace(r"$-$", "-").replace("--", "-").replace("\u2212", "-")
    cell = re.sub(r"\\(num|textbf|texttt|emph|SI)\{", "{", cell)
    m = re.search(r"-?\d+\.?\d*", cell)
    return float(m.group()) if m else None


def policy_key(cell):
    cell = re.sub(r"\\[a-zA-Z]+", " ", cell).replace("\\_", "_")
    cell = cell.replace("{", " ").replace("}", " ").strip().lower()
    for name in ("ns_unbounded", "ns_estimate_only", "ns_symbolic_only", "ns_neural_only",
                 "ppo_unmasked", "ppo_wp6", "rule", "ppo"):
        if name in cell:
            return name
    if "bounded" in cell or "nsshort" in cell or cell.startswith("ns"):
        return "ns"
    return None


def tex_tables(tex):
    out = {}
    for block in re.findall(r"\\begin\{table\}(.*?)\\end\{table\}", tex, re.S):
        m = re.search(r"\\label\{(tab:[^}]+)\}", block)
        if not m:
            continue
        body = block.split(r"\midrule", 1)[-1].split(r"\bottomrule")[0]
        out[m.group(1)] = [ln.strip() for ln in body.split(r"\\") if ln.strip()]
    return out


class Check:
    def __init__(self):
        self.ok = self.bad = 0
        self.failures = []

    def __call__(self, what, claimed, actual, tol):
        if claimed is None:
            return
        if abs(claimed - actual) <= tol:
            self.ok += 1
        else:
            self.bad += 1
            self.failures.append(f"{what}: thesis {claimed}, data {actual:.4g}")


def load(csv_path):
    rows = []
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            for k in ("score", "delivered_units", "lost_units", "energy_wh",
                      "violation_count", "wh_per_unit", "charge_trips",
                      "min_battery_percent", "decision_ms"):
                r[k] = float(r[k])
            r["seed"] = int(r["seed"])
            rows.append(r)
    return rows


def check_tables(chk, tex, rows):
    tables = tex_tables(tex)
    by = {}
    for r in rows:
        by.setdefault(r["policy"], []).append(r)
    ref = {(r["scenario"], r["seed"]): r["score"] for r in by["ns"]}

    print("Table 7.1, overall comparison (360 shifts per policy)")
    for line in tables["tab:overall"]:
        cells = [c.strip() for c in line.split("&")]
        pol = policy_key(cells[0])
        if pol not in by:
            print(f"  ?? unmatched row {cells[0][:40]}")
            continue
        sel = by[pol]
        scores = [r["score"] for r in sel]
        lo, hi = bootstrap_ci(scores)
        pairs = [(r["score"], ref[(r["scenario"], r["seed"])]) for r in sel]
        p = wilcoxon([a for a, _ in pairs], [b for _, b in pairs])
        unsafe = sum(1 for r in sel if r["violation_count"] > 0)

        chk(f"7.1 {pol} score", num(cells[1]), statistics.fmean(scores), 0.005)
        ci = re.search(r"\[([-\d.]+),\s*([-\d.]+)\]", cells[1])
        if ci:
            chk(f"7.1 {pol} CI low", float(ci.group(1)), lo, 0.05)
            chk(f"7.1 {pol} CI high", float(ci.group(2)), hi, 0.05)
        chk(f"7.1 {pol} delivered", num(cells[2]),
            statistics.fmean(r["delivered_units"] for r in sel), 0.05)
        chk(f"7.1 {pol} lost", num(cells[3]),
            statistics.fmean(r["lost_units"] for r in sel), 0.005)
        chk(f"7.1 {pol} Wh", num(cells[4]),
            statistics.fmean(r["energy_wh"] for r in sel), 0.005)
        chk(f"7.1 {pol} unsafe shifts", num(cells[5]), unsafe, 0)
        if pol != "ns":
            chk(f"7.1 {pol} delta", num(cells[6]),
                statistics.fmean(a - b for a, b in pairs), 0.005)
            claimed_p = cells[7]
            if "<" in claimed_p:
                chk.ok += 1 if p < 0.001 else 0
                if p >= 0.001:
                    chk.bad += 1
                    chk.failures.append(f"7.1 {pol} p: thesis p<0.001, data p={p:.4g}")
            else:
                chk(f"7.1 {pol} p", num(claimed_p), p, 0.0005)
        print(f"  {pol:18s} {statistics.fmean(scores):6.2f} [{lo:5.1f},{hi:5.1f}]  "
              f"unsafe {unsafe:3d}/360  p={p:.3g}")

    print("\nTable 7.2, per scenario (30 seeds each)")
    for line in tables["tab:per-scenario"]:
        cells = [c.strip() for c in line.split("&")]
        scen = re.sub(r"\\midrule", " ", cells[0])
        scen = re.sub(r"\\[a-zA-Z]+|[{}\\]", "", scen).replace(" ", "")
        if scen not in {r["scenario"] for r in rows}:
            print(f"  ?? unknown scenario {cells[0]}")
            continue
        actual_split = {r["split"] for r in rows if r["scenario"] == scen}.pop()
        if cells[1].strip() == actual_split:
            chk.ok += 1
        else:
            chk.bad += 1
            chk.failures.append(f"7.2 {scen} split: thesis {cells[1]}, data {actual_split}")
        out = [f"  {scen:20s} {actual_split:6s}"]
        for pol, cell in zip(COLS, cells[2:6]):
            sel = [r for r in rows if r["policy"] == pol and r["scenario"] == scen]
            mean = statistics.fmean(r["score"] for r in sel)
            chk(f"7.2 {scen}/{pol}", num(cell), mean, 0.005)
            unsafe = sum(1 for r in sel if r["violation_count"] > 0)
            m = re.search(r"\((\d+)\s*unsafe\)", cell)
            if m:
                chk(f"7.2 {scen}/{pol} unsafe", float(m.group(1)), unsafe, 0)
            elif unsafe:
                chk.bad += 1
                chk.failures.append(f"7.2 {scen}/{pol}: {unsafe} unsafe shifts, unmarked")
            if pol != "ns":
                pairs = [(r["score"], ref[(r["scenario"], r["seed"])]) for r in sel]
                p = wilcoxon([a for a, _ in pairs], [b for _, b in pairs])
                starred = "^{*}" in cell
                if starred == (p < 0.05):
                    chk.ok += 1
                else:
                    chk.bad += 1
                    chk.failures.append(
                        f"7.2 {scen}/{pol}: table {'marks' if starred else 'omits'} "
                        f"significance, p={p:.4g}")
            out.append(f"{mean:7.2f}")
        print(" ".join(out))
    return by, ref


def check_prose(chk, tex, by):
    print("\nProse claims in chapter 7")
    # "highest score among the safe policies in N of the twelve scenarios" follows
    # tab:per-scenario, so it ranges over that table's four columns, and "safe" is
    # judged per scenario, as the table marks unsafe entries scenario by scenario.
    wins = []
    for scen in sorted({r["scenario"] for r in by["ns"]}):
        cand = {}
        for pol in COLS:
            sel = [r for r in by[pol] if r["scenario"] == scen]
            if not any(r["violation_count"] > 0 for r in sel):
                cand[pol] = statistics.fmean(r["score"] for r in sel)
        if max(cand, key=cand.get) == "ns":
            wins.append(scen)
    m = re.search(r"highest score among the safe policies in (\w+) of the twelve", tex)
    if m:
        chk("7.2 ns wins among safe policies", float(WORDS.get(m.group(1), -1)), len(wins), 0)
    print(f"  ns best safe policy of table 7.2 in {len(wins)}/12: {', '.join(wins)}")

    clean = [p for p in by if not any(r["violation_count"] > 0 for r in by[p])]
    print(f"  globally violation-free policies: {len(clean)} ({', '.join(sorted(clean))}) "
          f"= {sum(len(by[p]) for p in clean)} clean shifts")

    m = re.search(r"\\SI\{([\d.]+)\}\{\\watt\\hour\} per\s+delivered unit", tex)
    if m:
        chk("7.6 rule Wh/unit", float(m.group(1)),
            statistics.fmean(r["wh_per_unit"] for r in by["rule"]), 0.0005)
    m = re.search(r"against \\SI\{([\d.]+)\}\{\\watt\\hour\} for the", tex)
    if m:
        chk("7.6 ns Wh/unit", float(m.group(1)),
            statistics.fmean(r["wh_per_unit"] for r in by["ns"]), 0.0005)
    for pol, claimed in [("rule", 7.1), ("ns", 2.8)]:
        chk(f"7.6 {pol} charge trips", claimed,
            statistics.fmean(r["charge_trips"] for r in by[pol]), 0.05)
    for pol, claimed in [("ppo", 45.0), ("ns", 59.0)]:
        chk(f"7.6 {pol} mean min battery", claimed,
            statistics.fmean(r["min_battery_percent"] for r in by[pol]), 0.05)
    print(f"  Wh/unit: rule {statistics.fmean(r['wh_per_unit'] for r in by['rule']):.3f}, "
          f"ns {statistics.fmean(r['wh_per_unit'] for r in by['ns']):.3f}; "
          f"charge trips {statistics.fmean(r['charge_trips'] for r in by['rule']):.2f} "
          f"vs {statistics.fmean(r['charge_trips'] for r in by['ns']):.2f}")

    print("\nSection 7.7, decision latency")
    for pol, claimed in [("ns", 1.46), ("ns_symbolic_only", 1.39), ("ppo", 2.67)]:
        actual = statistics.fmean(r["decision_ms"] for r in by[pol])
        chk(f"7.7 {pol} decision ms", claimed, actual, 0.005)
        print(f"  {pol:18s} {actual:.3f} ms")


def check_gazebo(chk, tex, results):
    print("\nSection 7.8, validation against the full simulation")
    rows = list(csv.DictReader(open(results / "gazebo_20260917_150533.csv")))
    pairs = {}
    for r in rows:
        pairs.setdefault((r["model"], r["scenario"], r["seed"]), {})[r["source"]] = r
    complete = [v for v in pairs.values() if len(v) == 2]
    usable = [v for v in complete
              if float(v["gazebo"]["shift_s"]) > 0.5 * float(v["sim"]["shift_s"])]
    print(f"  {len(pairs)} paired shifts, {len(usable)} ran to completion")
    m = re.search(r"Over the (\w+) shifts that ran to completion", tex)
    if m:
        chk("7.8 completed shifts", float(WORDS.get(m.group(1), -1)), len(usable), 0)

    for key in ("energy_wh", "delivered_units", "score"):
        g = statistics.fmean(float(v["gazebo"][key]) for v in usable)
        s = statistics.fmean(float(v["sim"][key]) for v in usable)
        print(f"  {key:16s} gazebo {g:7.2f}   sim {s:7.2f}   {100 * (g - s) / s:+.1f} %")

    m = re.search(r"\\SI\{([\d.]+)\}\{\\watt\\hour\} against \\SI\{([\d.]+)\}\{\\watt\\hour\}"
                  r"\s*\(\\SI\{([-+\d.]+)\}\{\\percent\}\)", tex)
    if m:
        chk("7.8 gazebo Wh", float(m.group(1)),
            statistics.fmean(float(v["gazebo"]["energy_wh"]) for v in usable), 0.006)
        chk("7.8 sim Wh", float(m.group(2)),
            statistics.fmean(float(v["sim"]["energy_wh"]) for v in usable), 0.006)
    m = re.search(r"\\num\{([\d.]+)\}\s+delivered units against \\num\{([\d.]+)\}", tex)
    if m:
        chk("7.8 gazebo delivered", float(m.group(1)),
            statistics.fmean(float(v["gazebo"]["delivered_units"]) for v in usable), 0.05)
        chk("7.8 sim delivered", float(m.group(2)),
            statistics.fmean(float(v["sim"]["delivered_units"]) for v in usable), 0.05)
    m = re.search(r"score \\num\{([\d.]+)\} against \\num\{([\d.]+)\}", tex)
    if m:
        chk("7.8 gazebo score", float(m.group(1)),
            statistics.fmean(float(v["gazebo"]["score"]) for v in usable), 0.006)
        chk("7.8 sim score", float(m.group(2)),
            statistics.fmean(float(v["sim"]["score"]) for v in usable), 0.006)
    viol = sum(int(v[s]["violation_count"]) for v in usable for s in ("gazebo", "sim"))
    chk("7.8 violations in completed shifts", 0.0, viol, 0)

    long_rows = {r["model"]: r for r in csv.DictReader(open(results / "gazebo_60min.csv"))
                 if r["source"] == "gazebo"}
    block = re.search(r"\\label\{tab:longshift\}(.*?)\\end\{tabular\}", tex, re.S).group(1)
    body = block.split(r"\midrule", 1)[-1].split(r"\bottomrule")[0]
    for line in [l for l in body.split(r"\\") if l.strip()]:
        cells = [c.strip() for c in line.split("&")]
        pol = "ns" if "NSshort" in cells[0] else "ppo"
        r = long_rows[pol]
        chk(f"7.8 60min {pol} score", num(cells[1]), float(r["score"]), 0.005)
        chk(f"7.8 60min {pol} delivered", num(cells[2]), float(r["delivered_units"]), 0)
        chk(f"7.8 60min {pol} Wh", num(cells[3]), float(r["energy_wh"]), 0.006)
        chk(f"7.8 60min {pol} min battery", num(cells[4]),
            float(r["min_battery_percent"]), 0.05)
        chk(f"7.8 60min {pol} violations", num(cells[5]), float(r["violation_count"]), 0)
        print(f"  60 min {pol:4s} score {float(r['score']):6.2f}  "
              f"delivered {r['delivered_units']:>3s}  "
              f"min battery {r['min_battery_percent']:>5s} %  "
              f"violations {r['violation_count']}")


def check_config(chk, thesis, root):
    print("\nConstants quoted in chapters 3, 5 and 6, against the configuration")
    import yaml
    params = yaml.safe_load((root / "src/robofetch_factory/config/params.yaml").read_text())
    mission = params["mission"]
    ev = mission["evaluation"]
    src = {f: (thesis / "body" / f).read_text()
           for f in ("03_problem.tex", "05_decision_models.tex", "06_experimental_setup.tex")}

    n_all = len(list((root / "src/robofetch_factory/config/scenarios").glob("*.yaml")))
    n_train = len(ev["training_scenarios"])
    checks = [
        ("seeds per scenario", ev["seeds"], "06_experimental_setup.tex",
         r"\\num\{(\d+)\} seeds"),
        ("first test seed", ev["test_seed_start"], "06_experimental_setup.tex",
         r"Test shifts use seeds \\numrange\{(\d+)\}"),
        ("last test seed", ev["test_seed_start"] + ev["seeds"] - 1,
         "06_experimental_setup.tex", r"Test shifts use seeds \\numrange\{\d+\}\{(\d+)\}"),
        ("total shifts", n_all * ev["seeds"] * 9, "06_experimental_setup.tex",
         r"\\num\{(3240)\}"),
        ("battery reserve percent", params["robot"]["battery"]["reserve_percent"],
         "03_problem.tex",
         r"at least \\SI\{(\d+)\}\{\\percent\} of the battery must remain"),
        ("safety violation penalty", mission["objective"]["cost_per_safety_violation"],
         "03_problem.tex", r"penalty worth (twenty) delivered units"),
        ("energy weight", mission["objective"]["cost_per_wh"],
         "03_problem.tex", r"half a delivered unit|\\num\{(0\.5)\}"),
    ]
    for name, value, fname, pat in checks:
        m = re.search(pat, src[fname])
        found = next((g for g in (m.groups() if m else ()) if g), None)
        if found == "twenty":
            found = "20"
        if m and found is None and "half a delivered unit" in m.group(0):
            found = "0.5"
        if found is None:
            print(f"  -- {name}: config says {value}; no matching phrase in the text")
            continue
        chk(f"cfg {name}", float(found), float(value), 1e-9)
        print(f"  {name}: config {value}, thesis {found}")
    print(f"  scenarios on disk {n_all}, training {n_train}, unseen {n_all - n_train}")
    chk("cfg total scenarios", 12.0, float(n_all), 0)
    chk("cfg training scenarios", 6.0, float(n_train), 0)


def check_macros(chk, thesis, rows, results, root):
    """macros.tex is the thesis's single source for repeated numbers: check each one."""
    import yaml
    print("\nmacros.tex, the single source of every repeated number")
    src = (thesis / "macros.tex").read_text()
    defs = dict(re.findall(r"\\newcommand\{\\(\w+)\}\{([^}]*)\}", src))

    by = {}
    for r in rows:
        by.setdefault(r["policy"], []).append(r)

    def mean(pol, key="score", split=None, scen=None):
        sel = [r for r in by[pol]
               if (split is None or r["split"] == split)
               and (scen is None or r["scenario"] == scen)]
        return statistics.fmean(r[key] for r in sel)

    pooled = {"scoreNS": "ns", "scoreSymbolic": "ns_symbolic_only",
              "scoreUnbounded": "ns_unbounded", "scoreEstimateOnly": "ns_estimate_only",
              "scoreNeuralOnly": "ns_neural_only", "scorePPO": "ppo",
              "scoreRule": "rule", "scorePPOunmasked": "ppo_unmasked"}
    for macro, pol in pooled.items():
        chk(f"macro \\{macro}", float(defs[macro]), mean(pol), 0.005)

    unseen = {"unseenNS": "ns", "unseenSymbolic": "ns_symbolic_only",
              "unseenPPO": "ppo", "unseenUnbounded": "ns_unbounded",
              "unseenNeuralOnly": "ns_neural_only"}
    for macro, pol in unseen.items():
        chk(f"macro \\{macro}", float(defs[macro]), mean(pol, split="unseen"), 0.005)
    for macro, pol in {"seenNS": "ns", "seenPPO": "ppo",
                       "seenNeuralOnly": "ns_neural_only"}.items():
        chk(f"macro \\{macro}", float(defs[macro]), mean(pol, split="seen"), 0.005)
    print(f"  seen ns {mean('ns', split='seen'):.2f} / unseen {mean('ns', split='unseen'):.2f}; "
          f"unseen ppo {mean('ppo', split='unseen'):.2f}, "
          f"unbounded {mean('ns_unbounded', split='unseen'):.2f}")

    for macro, pol in {"heavyBounded": "ns", "heavyUnbounded": "ns_unbounded"}.items():
        chk(f"macro \\{macro}", float(defs[macro]),
            mean(pol, scen="heavy_load"), 0.005)
    print(f"  heavy_load: bounded {mean('ns', scen='heavy_load'):.2f}, "
          f"unbounded {mean('ns_unbounded', scen='heavy_load'):.2f}")

    for macro, pol in {"violNS": "ns", "violPPO": "ppo",
                       "violNeuralOnly": "ns_neural_only"}.items():
        claimed = float(defs[macro].split("/")[0])
        chk(f"macro \\{macro}", claimed,
            sum(1 for r in by[pol] if r["violation_count"] > 0), 0)

    params = yaml.safe_load((root / "src/robofetch_factory/config/params.yaml").read_text())
    chk("macro \\maxCorrection vs params.yaml", float(defs["maxCorrection"]),
        float(params["mission"]["neurosymbolic"]["max_correction"]), 1e-9)
    print(f"  maxCorrection: macro {defs['maxCorrection']}, "
          f"params.yaml {params['mission']['neurosymbolic']['max_correction']}")

    g = list(csv.DictReader(open(results / "gazebo_20260917_150533.csv")))
    pairs = {}
    for r in g:
        pairs.setdefault((r["model"], r["scenario"], r["seed"]), {})[r["source"]] = r
    usable = [v for v in pairs.values() if len(v) == 2
              and float(v["gazebo"]["shift_s"]) > 0.5 * float(v["sim"]["shift_s"])]
    for macro, key in [("gazeboEnergyGap", "energy_wh"), ("gazeboScoreGap", "score")]:
        gz = statistics.fmean(float(v["gazebo"][key]) for v in usable)
        sm = statistics.fmean(float(v["sim"][key]) for v in usable)
        chk(f"macro \\{macro}", float(defs[macro].replace("\\,\\%", "").replace("+", "")),
            100 * (gz - sm) / sm, 0.05)

    long_rows = {r["model"]: r for r in csv.DictReader(open(results / "gazebo_60min.csv"))
                 if r["source"] == "gazebo"}
    for macro, pol, key in [("longShiftNS", "ns", "score"), ("longShiftPPO", "ppo", "score"),
                            ("longShiftNSbattery", "ns", "min_battery_percent"),
                            ("longShiftPPObattery", "ppo", "min_battery_percent")]:
        chk(f"macro \\{macro}", float(defs[macro].replace("\\,\\%", "")),
            float(long_rows[pol][key]), 0.005)

    used = set(re.findall(r"\\([a-zA-Z]+)",
                          "".join(f.read_text() for f in sorted((thesis / "body").glob("*.tex")))))
    unused = [m for m in defs if m not in used and m not in ("file",)]
    print(f"  {len(defs)} macros defined, "
          f"{len([m for m in defs if m in used])} used in the body"
          + (f", unused: {', '.join(unused)}" if unused else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--thesis", default=str(ROOT.parent / "autonomous-industrial-mobility-thesis"),
                    help="path to the thesis checkout")
    ap.add_argument("--csv", default=str(ROOT / "tools/ai/results/episodes_20260920_210847.csv"))
    args = ap.parse_args()

    thesis = pathlib.Path(args.thesis)
    results = pathlib.Path(args.csv).parent
    rows = load(args.csv)
    tex = (thesis / "body/07_results.tex").read_text()
    chk = Check()

    print(f"data : {pathlib.Path(args.csv).name} ({len(rows)} shifts, "
          f"{len({r['policy'] for r in rows})} policies, "
          f"{len({r['scenario'] for r in rows})} scenarios)")
    print(f"tex  : {thesis}\n")

    by, _ = check_tables(chk, tex, rows)
    check_prose(chk, tex, by)
    check_gazebo(chk, tex, results)
    check_macros(chk, thesis, rows, results, ROOT)
    check_config(chk, thesis, ROOT)

    print(f"\n{chk.ok} checks passed, {chk.bad} failed")
    for f in chk.failures:
        print(f"  FAIL {f}")
    return 1 if chk.bad else 0


if __name__ == "__main__":
    sys.exit(main())
