"""The symbolic layer: every hard rule must actually forbid what it claims to forbid.

This is the guarantee the neuro-symbolic model gives that a purely learned policy cannot: no
matter what the network scores, the robot will not strand itself, cook its motors, overload
itself or keep working when it needs maintenance. Each rule gets a test that constructs the
situation and checks both the refusal AND its stated reason.
"""
import os

import pytest

from robofetch_core.mission_plan import CHARGE, DELIVER, PICKUP, WAIT, Action
from robofetch_factory.factory_model import load_config
from robofetch_ai.env.factory_sim import FactorySim
from robofetch_ai.policies.symbolic import SymbolicLayer

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robofetch_factory", "config")


def make(scenario="balanced", seed=0, **overrides):
    sim = FactorySim(scenario, seed, CONFIG_DIR, overrides=overrides or None)
    return sim, SymbolicLayer(sim.cfg)


def verdict(sim, rules, action):
    return rules.check(sim.state(), action, sim.p, sim.matrix)


# ------------------------------------------------------------------------- hard rules
def test_refuses_a_trip_that_would_strand_the_robot():
    sim, rules = make(**{"robot": {"battery": {"initial_percent": 16.0}}})
    v = verdict(sim, rules, Action(PICKUP, "B"))
    assert not v.allowed and "reserve" in v.why()


def test_allows_the_same_trip_with_a_full_battery():
    sim, rules = make()
    assert verdict(sim, rules, Action(PICKUP, "B")).allowed


def test_refuses_work_when_the_robot_needs_maintenance():
    sim, rules = make(**{"robot": {"wear": {"initial_percent": 20.0}}})
    for action in (Action(PICKUP, "B"), Action(DELIVER)):
        v = verdict(sim, rules, action)
        assert not v.allowed and "maintenance" in v.why()
    assert verdict(sim, rules, Action(CHARGE, value=90.0)).allowed
    assert verdict(sim, rules, Action(WAIT, value=60.0)).allowed


def test_refuses_a_pickup_that_does_not_fit():
    sim, rules = make()
    sim.step(Action(PICKUP, "B"))                  # fill up on B first
    while sim.units_that_fit("B") > 0:
        sim._advance(60.0)
        sim.step(Action(PICKUP, "B"))
    v = verdict(sim, rules, Action(PICKUP, "B"))
    assert not v.allowed and "fits" in v.why()


def test_refuses_a_route_that_would_overheat_the_motors():
    sim, rules = make(**{"robot": {"thermal": {"max_c": 25.0, "heat_c_per_s": 1.0}}})
    v = verdict(sim, rules, Action(PICKUP, "A"))
    assert not v.allowed and "C (limit" in v.why()


def test_hard_rules_use_the_robots_own_models():
    """The reserve check must follow the configured reserve, not a hard-coded number."""
    strict, strict_rules = make(**{"robot": {"battery": {"initial_percent": 30.0,
                                                         "reserve_percent": 28.0}}})
    relaxed, relaxed_rules = make(**{"robot": {"battery": {"initial_percent": 30.0,
                                                           "reserve_percent": 5.0}}})
    assert not verdict(strict, strict_rules, Action(PICKUP, "A")).allowed
    assert verdict(relaxed, relaxed_rules, Action(PICKUP, "A")).allowed


# --------------------------------------------------------------------- priority rules
def test_a_blocked_line_is_the_top_priority():
    sim, rules = make()
    while sim.sections["B"].status != "BLOCKED":
        sim._advance(30.0)
    v = verdict(sim, rules, Action(PICKUP, "B"))
    assert v.allowed and v.tier == 2 and "BLOCKED" in v.why()


def test_a_line_about_to_block_outranks_a_quiet_one():
    sim, rules = make()
    while sim.sections["B"].time_to_full_sim_s() > 120:
        sim._advance(30.0)
    b = verdict(sim, rules, Action(PICKUP, "B"))
    c = verdict(sim, rules, Action(WAIT, value=60.0))
    assert b.tier > c.tier and "fills in" in b.why()


def test_an_empty_battery_makes_charging_top_priority():
    sim, rules = make(**{"robot": {"battery": {"initial_percent": 16.0}}})
    v = verdict(sim, rules, Action(CHARGE, value=90.0))
    assert v.allowed and v.tier == 2 and "reserve" in v.why()


def test_every_verdict_explains_itself():
    sim, rules = make()
    for action in sim.legal_actions():
        v = rules.check(sim.state(), action, sim.p, sim.matrix)
        assert v.why() and v.why() != "routine" or action.kind == WAIT


def test_allowed_by_tier_keeps_only_the_most_urgent():
    sim, rules = make()
    while sim.sections["B"].status != "BLOCKED":
        sim._advance(30.0)
    verdicts = rules.evaluate(sim.state(), sim.legal_actions(), sim.p, sim.matrix)
    top, tier = SymbolicLayer.allowed_by_tier(verdicts)
    assert tier == 2 and all(str(a) == "PICKUP:B" or v.tier == 2 for a, v in top.items())
