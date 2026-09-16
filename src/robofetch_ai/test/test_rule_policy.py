"""The rule-based reference policy: sane, safe and explainable in every scenario."""
import os

import pytest

from robofetch_ai.env.factory_sim import run_episode
from robofetch_ai.policies.rule_based import RuleBasedPolicy

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robofetch_factory", "config")
SCENARIOS = sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_delivers_and_never_violates_safety(scenario):
    summary, _ = run_episode(RuleBasedPolicy(), scenario, 0, CONFIG_DIR)
    assert summary["delivered_units"] > 0
    assert summary["violations"] == []
    assert summary["min_battery_percent"] >= 15.0      # the configured reserve


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_every_decision_is_explained(scenario):
    _, decisions = run_episode(RuleBasedPolicy(), scenario, 1, CONFIG_DIR, trace=True)
    assert decisions and all(d["why"] for d in decisions)


def test_serves_a_blocked_line_before_a_quiet_one():
    """With B blocked and A nearly empty the robot must go to B."""
    from robofetch_ai.env.factory_sim import FactorySim
    sim = FactorySim("balanced", 0, CONFIG_DIR)
    for _ in range(int(sim.sections["B"].time_to_full_sim_s()) + 60):
        sim._advance(1.0)
    assert sim.sections["B"].status == "BLOCKED"
    decision = RuleBasedPolicy().decide(sim.state(), sim.legal_actions(), sim)
    assert str(decision.action) == "PICKUP:B" and "BLOCKED" in decision.explanation


def test_charges_when_the_battery_only_just_covers_getting_home():
    """At 15.6 % the reserve (15 %) plus the drive home is all that is left: charge, do not work.

    (With a little more - e.g. 17 % - delivering first is correct and the policy does that; the
    first version of this test wrongly demanded charging there.)"""
    from robofetch_ai.env.factory_sim import FactorySim
    sim = FactorySim("balanced", 0, CONFIG_DIR,
                     overrides={"robot": {"battery": {"initial_percent": 15.6}}})
    sim.step(sim.legal_actions()[0])            # move away from the charger
    decision = RuleBasedPolicy().decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action.kind == "CHARGE"
    assert "battery" in decision.explanation


def test_delivers_first_when_the_battery_still_allows_it():
    from robofetch_ai.env.factory_sim import FactorySim
    sim = FactorySim("balanced", 0, CONFIG_DIR,
                     overrides={"robot": {"battery": {"initial_percent": 17.0}}})
    sim.step(sim.legal_actions()[0])
    decision = RuleBasedPolicy().decide(sim.state(), sim.legal_actions(), sim)
    assert decision.action.kind in ("DELIVER", "PICKUP")
