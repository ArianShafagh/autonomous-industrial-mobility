"""The neuro-symbolic policy: the rules must always win over the network, and every decision
must be explainable.

The point of the architecture is that the safety guarantee does not depend on what the network
learnt. These tests therefore include a DELIBERATELY BAD network (one that always prefers the
most dangerous action) and check that the robot still behaves safely.
"""
import os

import pytest
import torch

from robofetch_core.mission_plan import CHARGE, PICKUP, WAIT, Action
from robofetch_ai.env.factory_sim import FactorySim, run_episode
from robofetch_ai.policies.features import FEATURE_NAMES
from robofetch_ai.policies.neural import ActionScorer
from robofetch_ai.policies.neurosymbolic import NeuroSymbolicPolicy

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robofetch_factory", "config")
SCENARIOS = sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))


class AlwaysPicksTheWorst(ActionScorer):
    """A network that scores 'drive somewhere far with an empty battery' highest."""

    def score(self, rows):
        out = []
        for row in rows:
            f = dict(zip(FEATURE_NAMES, row))
            out.append(f["distance"] * 10.0 - f["battery_after_home"] * 100.0)
        return out


def sim_and_policy(mode="full", scenario="balanced", seed=0, scorer=None, **overrides):
    sim = FactorySim(scenario, seed, CONFIG_DIR, overrides=overrides or None)
    policy = NeuroSymbolicPolicy(sim.cfg, mode=mode, scorer=scorer or AlwaysPicksTheWorst())
    return sim, policy


# --------------------------------------------------------------- rules beat the network
def test_a_bad_network_cannot_break_the_battery_rule():
    sim, policy = sim_and_policy(**{"robot": {"battery": {"initial_percent": 17.0}}})
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action.kind in (CHARGE, WAIT)
    assert "reserve" in decision.explanation or "forbidden" in decision.explanation


def test_a_bad_network_cannot_make_the_robot_work_when_it_needs_maintenance():
    sim, policy = sim_and_policy(**{"robot": {"wear": {"initial_percent": 10.0}}})
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action.kind in (CHARGE, WAIT)


def test_a_whole_shift_with_a_bad_network_stays_safe():
    sim = FactorySim("low_battery_start", 0, CONFIG_DIR)
    policy = NeuroSymbolicPolicy(sim.cfg, mode="full", scorer=AlwaysPicksTheWorst())
    summary, _ = run_episode(policy, "low_battery_start", 0, CONFIG_DIR)
    assert summary["violations"] == []
    assert summary["min_battery_percent"] >= 15.0


def test_the_network_only_chooses_inside_the_top_priority_tier():
    sim, policy = sim_and_policy()
    while sim.sections["B"].status != "BLOCKED":
        sim._advance(30.0)
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert str(decision.action) == "PICKUP:B"          # tier 2 leaves nothing else
    assert "unblock a stopped line" in decision.explanation


# ------------------------------------------------------------------------------- modes
def test_neural_only_ignores_the_rules():
    """The ablation must really remove the guarantee - otherwise it proves nothing."""
    sim, policy = sim_and_policy(mode="neural",
                                 **{"robot": {"battery": {"initial_percent": 17.0}}})
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action.kind == PICKUP               # the bad network gets its way
    assert "no rules" in decision.explanation


def test_symbolic_only_needs_no_network():
    sim = FactorySim("balanced", 0, CONFIG_DIR)
    policy = NeuroSymbolicPolicy(sim.cfg, mode="symbolic")
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action in sim.legal_actions() and decision.explanation


def test_missing_model_falls_back_to_rules_and_says_so():
    sim = FactorySim("balanced", 0, CONFIG_DIR)
    policy = NeuroSymbolicPolicy(sim.cfg, mode="full", model_path="/nonexistent/model.pt")
    assert policy.mode == "symbolic"
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert "no trained scorer" in decision.explanation


# ------------------------------------------------------------------------ explanations
@pytest.mark.parametrize("scenario", SCENARIOS)
def test_every_decision_of_a_whole_shift_is_explained(scenario):
    sim = FactorySim(scenario, 1, CONFIG_DIR)
    policy = NeuroSymbolicPolicy(sim.cfg, mode="symbolic")
    _, decisions = run_episode(policy, scenario, 1, CONFIG_DIR, trace=True)
    assert decisions
    for d in decisions:
        assert d["why"] and ("[" in d["why"] or "forbidden" in d["why"])


def test_decisions_are_fast_enough_for_the_robot():
    sim = FactorySim("balanced", 0, CONFIG_DIR)
    policy = NeuroSymbolicPolicy(sim.cfg, mode="symbolic")
    decision = policy.decide(sim.state(), sim.legal_actions(), sim)
    assert decision.latency_ms < 100.0


# ----------------------------------------------------------------------------- scorer
def test_scorer_saves_and_loads_with_its_normalisation(tmp_path):
    model = ActionScorer((8, 8))
    x = torch.randn(64, len(FEATURE_NAMES))
    model.set_normalisation(x)
    before = model.score(x[:5].tolist())
    path = model.save(str(tmp_path / "scorer.pt"))
    after = ActionScorer.load(path).score(x[:5].tolist())
    assert before == pytest.approx(after, abs=1e-6)


def test_scorer_refuses_a_model_trained_on_other_features(tmp_path):
    path = str(tmp_path / "old.pt")
    torch.save({"state_dict": ActionScorer((8, 8)).state_dict(), "hidden_sizes": [8, 8],
                "features": ["something", "else"], "extra": {}}, path)
    with pytest.raises(ValueError, match="different features"):
        ActionScorer.load(path)
