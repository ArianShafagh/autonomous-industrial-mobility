#!/usr/bin/env python3
"""Train the neural scorer of the neuro-symbolic model.

Where the training signal comes from (no human labels, no hand-written "right answers"):

    At a decision point, for EVERY legal action, the simulator is cloned, that action is executed,
    and the shift is then played on for `rollout_horizon_s` by a REFERENCE policy. The discounted
    objective collected in that look-ahead is what the action was actually worth, and the network
    learns to predict it from the action's features.

The reference policy matters: a look-ahead that continues badly makes every action look equally
poor. So training runs as POLICY ITERATION - round 1 looks ahead with the symbolic policy, and
each further round looks ahead with the neuro-symbolic model trained in the previous round. The
target is always measured, never assumed, and always with the objective the thesis reports.

    venv/bin/python tools/ai/train_ns.py                       # settings from params.yaml
    venv/bin/python tools/ai/train_ns.py --iterations 2 --rollout-policy ns_symbolic_only
    venv/bin/python tools/ai/train_ns.py --episodes 30 --quick
"""
import argparse
import copy
import functools
import json
import os
import random
import sys
import time

import torch
from torch import nn

# Progress must be visible even when the output is redirected to a log file (Python buffers
# stdout when it is not a terminal, which made a 30-minute training run look completely silent).
print = functools.partial(print, flush=True)  # noqa: A001

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

from robofetch_factory.factory_model import load_config  # noqa: E402
from robofetch_ai.env.factory_sim import FactorySim, run_episode  # noqa: E402
from robofetch_ai.policies.features import (FEATURE_NAMES, action_features,  # noqa: E402
                                            symbolic_estimate)
from robofetch_ai.policies.neural import DEFAULT_MODEL, ActionScorer  # noqa: E402
from robofetch_ai.policies.neurosymbolic import NeuroSymbolicPolicy  # noqa: E402
from robofetch_ai.policies.rule_based import RuleBasedPolicy  # noqa: E402
from robofetch_ai.policies.symbolic import SymbolicLayer  # noqa: E402

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")
RESULTS_DIR = os.path.join(WS, "tools", "ai", "results")


def _one_rollout(sim, action, horizon_s, gamma_per_100s, policy, rng=None):
    clone = copy.deepcopy(sim)
    if rng is not None:
        # A different possible future: same state, fresh production and fault randomness.
        for sid, section in clone.sections.items():
            section.rng = random.Random(f"{rng}:{sid}:{clone.time_s}")
    start = clone.time_s
    outcome = clone.step(action)
    total = clone.reward(outcome)
    while not clone.done and clone.time_s - start < horizon_s:
        state, legal = clone.state(), clone.legal_actions()
        nxt = policy.decide(state, legal, clone).action
        outcome = clone.step(nxt)
        total += gamma_per_100s ** ((clone.time_s - start) / 100.0) * clone.reward(outcome)
    return total


def rollout_value(sim, action, horizon_s, gamma_per_100s, policy, samples=1, seed=0):
    """Value of taking `action` now and then behaving reasonably for `horizon_s`.

    Production and faults are random, so one look-ahead is a noisy measurement of an action's
    worth; `samples` different futures are averaged instead.
    """
    if samples <= 1:
        return _one_rollout(sim, action, horizon_s, gamma_per_100s, policy)
    values = [_one_rollout(sim, action, horizon_s, gamma_per_100s, policy, rng=seed + i)
              for i in range(samples)]
    return sum(values) / len(values)


def collect(cfg, scenarios, episodes, horizon_s, gamma, exploration, seed, samples=1,
            rollout_policy=None, log_every=10):
    """Walk shifts and label the decisions the model ACTUALLY has to make.

    Two things matter here:

    * **Whose states.** The shifts are walked by the current reference policy (round 1: the
      symbolic policy, later rounds: the model trained so far), not by the oracle. A network
      trained on someone else's states is asked at run time about situations it never saw - which
      is exactly why the first version lost to the rules.
    * **Which actions.** Only the candidates the symbolic layer allows in the top priority tier
      are valued and learnt, because those are the only ones the model ever chooses between.
      States with a single candidate teach nothing and are skipped.
    """
    rng = random.Random(seed)
    reference = rollout_policy or RuleBasedPolicy()
    rules = SymbolicLayer(cfg)
    rows, targets, meta, estimates = [], [], [], []
    point_id = 0
    skipped = 0
    started = time.time()
    for episode in range(episodes):
        scenario = scenarios[episode % len(scenarios)]
        sim = FactorySim(scenario, seed + episode, CONFIG_DIR)
        while not sim.done:
            state, legal = sim.state(), sim.legal_actions()
            verdicts = rules.evaluate(state, legal, sim.p, sim.matrix)
            candidates, _ = rules.allowed_by_tier(verdicts)
            candidates = list(candidates) or legal
            if len(candidates) < 2:
                skipped += 1
                sim.step(reference.decide(state, legal, sim).action)
                continue
            values = {a: rollout_value(sim, a, horizon_s, gamma, reference, samples,
                                       seed + 1000 * point_id) for a in candidates}
            for action, value in values.items():
                estimate = symbolic_estimate(state, action, sim.p, sim.matrix, sim.objective)
                rows.append(action_features(state, action, sim.p, sim.matrix))
                # The network learns the CORRECTION to the symbolic estimate, not the raw value.
                targets.append(value - estimate)
                estimates.append(estimate)
                meta.append((scenario, str(action), point_id))
            point_id += 1
            # Follow the reference policy (with a little exploration) so the next state is one the
            # model itself would reach.
            chosen = (rng.choice(candidates) if rng.random() < exploration
                      else reference.decide(state, legal, sim).action)
            sim.step(chosen)
        if (episode + 1) % log_every == 0:
            print(f"  {episode + 1}/{episodes} shifts, {len(rows)} samples from {point_id} "
                  f"decisions ({skipped} had only one option), {time.time() - started:.0f} s")
    return rows, targets, meta, estimates


def train(model, x, y, epochs, batch_size, lr, validation_fraction, seed):
    torch.manual_seed(seed)
    n_val = max(1, int(len(x) * validation_fraction))
    perm = torch.randperm(len(x))
    val, train_idx = perm[:n_val], perm[n_val:]
    x_train, y_train, x_val, y_val = x[train_idx], y[train_idx], x[val], y[val]
    model.set_normalisation(x_train)
    optimiser = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    history = []
    for epoch in range(epochs):
        model.train()
        order = torch.randperm(len(x_train))
        total = 0.0
        for start in range(0, len(x_train), batch_size):
            batch = order[start:start + batch_size]
            optimiser.zero_grad()
            loss = loss_fn(model(x_train[batch]), y_train[batch])
            loss.backward()
            optimiser.step()
            total += loss.item() * len(batch)
        model.eval()
        with torch.no_grad():
            val_loss = loss_fn(model(x_val), y_val).item()
        history.append({"epoch": epoch + 1, "train_mse": total / len(x_train), "val_mse": val_loss})
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"  epoch {epoch + 1:3d}  train MSE {history[-1]['train_mse']:.3f}  "
                  f"val MSE {val_loss:.3f}")
    with torch.no_grad():
        pred = model(x_val)
        ss_res = ((y_val - pred) ** 2).sum().item()
        ss_tot = ((y_val - y_val.mean()) ** 2).sum().item()
    return history, {"val_mse": history[-1]["val_mse"],
                     "val_r2": 1 - ss_res / ss_tot if ss_tot else float("nan")}


def train_ranking(model, x, y, estimates, meta, epochs, lr, validation_fraction, seed,
                  value_weight=0.2, temperature=1.0):
    """Train the scorer to RANK the candidates of a decision, not to regress their value.

    The look-ahead value of an action is noisy (production and faults are random), and the
    candidates of one decision often differ by far less than that noise - so regressing values
    spends the whole network on noise. What the model actually has to get right is "which of these
    is better", so the loss is a softmax cross-entropy over the candidates of each decision point,
    with the look-ahead's best action as the target. A small value-regression term is kept so the
    scores stay on a meaningful scale (and remain readable in the explanation).
    """
    torch.manual_seed(seed)
    groups = {}
    for i, (_, _, point_id) in enumerate(meta):
        groups.setdefault(point_id, []).append(i)
    points = sorted(groups)
    random.Random(seed).shuffle(points)
    n_val = max(1, int(len(points) * validation_fraction))
    val_points, train_points = points[:n_val], points[n_val:]

    model.set_normalisation(x[[i for p in train_points for i in groups[p]]])
    optimiser = torch.optim.Adam(model.parameters(), lr=lr)
    history = []

    def evaluate(point_ids):
        model.eval()
        with torch.no_grad():
            hits = 0
            for point in point_ids:
                idx = groups[point]
                total = model(x[idx]) + estimates[idx]
                hits += int(torch.argmax(total).item() == torch.argmax(y[idx] + estimates[idx]).item())
        return hits / len(point_ids)

    for epoch in range(epochs):
        model.train()
        random.Random(seed + epoch).shuffle(train_points)
        total_loss = 0.0
        for point in train_points:
            idx = groups[point]
            optimiser.zero_grad()
            correction = model(x[idx])
            scores = (correction + estimates[idx]) / temperature
            target = torch.argmax(y[idx] + estimates[idx])
            loss = nn.functional.cross_entropy(scores.unsqueeze(0), target.unsqueeze(0))
            loss = loss + value_weight * nn.functional.mse_loss(correction, y[idx])
            loss.backward()
            optimiser.step()
            total_loss += loss.item()
        val_top1 = evaluate(val_points)
        history.append({"epoch": epoch + 1, "loss": total_loss / len(train_points),
                        "val_top1": val_top1})
        if (epoch + 1) % 5 == 0 or epoch == 0:
            print(f"  epoch {epoch + 1:3d}  loss {history[-1]['loss']:.3f}  "
                  f"picks the look-ahead's best action {100 * val_top1:.1f} % of the time")
    return history, {"val_top1": history[-1]["val_top1"],
                     "train_top1": evaluate(train_points[:len(val_points)])}


def agreement_with_oracle(model, rows, targets, meta):
    """How often the trained scorer prefers the action the look-ahead says is best."""
    points = {}
    for i, (_, _, point_id) in enumerate(meta):
        points.setdefault(point_id, []).append(i)
    scores = model.score(rows)
    hits = sum(max(idx, key=lambda i: scores[i]) == max(idx, key=lambda i: targets[i])
               for idx in points.values())
    return hits / len(points) if points else float("nan")


def evaluate_model(cfg, scorer, scenarios, seeds=4):
    """Mean score over a small set of validation shifts - how good this round actually is."""
    scores = []
    for scenario in scenarios:
        for seed in range(1000, 1000 + seeds):      # seeds unseen during collection
            policy = NeuroSymbolicPolicy(cfg, mode="full", scorer=scorer)
            summary, _ = run_episode(policy, scenario, seed, CONFIG_DIR)
            scores.append(summary["score"])
    return sum(scores) / len(scores)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=None)
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--out", default=DEFAULT_MODEL)
    ap.add_argument("--quick", action="store_true", help="short horizon and few epochs (smoke test)")
    ap.add_argument("--iterations", type=int, default=None,
                    help="policy-iteration rounds (default: from params.yaml)")
    ap.add_argument("--rollout-policy", default="ns_symbolic_only",
                    choices=["rule", "ns_symbolic_only"],
                    help="what the look-ahead uses in the FIRST round")
    args = ap.parse_args()

    cfg = load_config("balanced", CONFIG_DIR)
    ns = cfg["mission"]["neurosymbolic"]
    t = dict(ns["training"])
    if args.episodes:
        t["episodes"] = args.episodes
    if args.quick:
        t.update(episodes=min(t["episodes"], 6), rollout_horizon_s=300.0, epochs=10,
                 rollout_samples=1)
    scenarios = args.scenarios or sorted(f[:-5] for f in
                                         os.listdir(os.path.join(CONFIG_DIR, "scenarios")))

    iterations = args.iterations or int(t.get("iterations", 1))
    reference = (RuleBasedPolicy() if args.rollout_policy == "rule"
                 else NeuroSymbolicPolicy(cfg, mode="symbolic"))
    model = path = None
    best = (float("-inf"), 0)
    all_metrics = []

    for round_no in range(1, iterations + 1):
        print(f"\n=== round {round_no}/{iterations}: look-ahead continues with "
              f"'{getattr(reference, 'name', reference)}'")
        print(f"collecting: {t['episodes']} shifts over {scenarios}, "
              f"horizon {t['rollout_horizon_s']:.0f} s, {t.get('rollout_samples', 1)} futures each")
        started = time.time()
        rows, targets, meta, estimate_rows = collect(
            cfg, scenarios, t["episodes"], t["rollout_horizon_s"], t["discount_per_100s"],
            t["exploration"], t["seed"] + round_no, samples=t.get("rollout_samples", 1),
            rollout_policy=reference)
        collect_s = time.time() - started
        print(f"{len(rows)} samples in {collect_s:.0f} s; training the scorer")

        x = torch.tensor(rows, dtype=torch.float32)
        y = torch.tensor(targets, dtype=torch.float32)
        estimates = torch.tensor(estimate_rows, dtype=torch.float32)
        model = ActionScorer(tuple(ns["hidden_sizes"]))
        history, metrics = train_ranking(model, x, y, estimates, meta, t["epochs"],
                                         t["learning_rate"], t["validation_fraction"], t["seed"])
        metrics.update(round=round_no, samples=len(rows), collect_seconds=round(collect_s, 1),
                       rollout_policy=getattr(reference, "name", str(reference)),
                       training_config=t, scenarios=scenarios)
        metrics["validation_score"] = evaluate_model(cfg, model, scenarios)
        all_metrics.append(metrics)
        print(f"round {round_no}: picks the look-ahead's best action "
              f"{100 * metrics['val_top1']:.1f} % of the time (validation), "
              f"validation score {metrics['validation_score']:.2f}")
        # Keep the round that actually performs best, not simply the last one.
        if metrics["validation_score"] >= best[0]:
            best = (metrics["validation_score"], round_no)
            path = model.save(args.out, extra=metrics)
            print(f"  -> best so far, saved")
        # Next round looks ahead with the model just trained.
        reference = NeuroSymbolicPolicy(cfg, mode="full", scorer=model)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    report = os.path.join(RESULTS_DIR, time.strftime("train_ns_%Y%m%d_%H%M%S.json"))
    with open(report, "w") as fh:
        json.dump({"rounds": all_metrics, "history": history, "features": FEATURE_NAMES}, fh,
                  indent=2)
    print(f"\nbest round: {best[1]} (validation score {best[0]:.2f})")
    print(f"model  -> {os.path.relpath(path, WS)}")
    print(f"report -> {os.path.relpath(report, WS)}")


if __name__ == "__main__":
    main()
