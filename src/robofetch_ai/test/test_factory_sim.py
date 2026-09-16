"""System tests for the fast simulator: it must behave like the real system, not merely run.

Pinned here: nothing is created or lost (units are conserved), the energy is the robot model's
energy, actions do what they claim, safety violations are recorded, and a seed reproduces a shift.
"""
import os

import pytest

from robofetch_core.mission_plan import CHARGE, DELIVER, PICKUP, WAIT, Action
from robofetch_ai.env.factory_sim import FactorySim, run_episode
from robofetch_ai.policies.rule_based import RuleBasedPolicy

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robofetch_factory", "config")


def sim(scenario="balanced", seed=0, **kw):
    return FactorySim(scenario, seed, CONFIG_DIR, **kw)


# ------------------------------------------------------------------------ bookkeeping
def test_units_are_conserved():
    s = sim()
    run_episode(RuleBasedPolicy(), "balanced", 0, CONFIG_DIR)
    policy = RuleBasedPolicy()
    while not s.done:
        s.step(policy.decide(s.state(), s.legal_actions(), s).action)
    produced = sum(sec.produced_total for sec in s.sections.values())
    initial = sum(sec.params.initial_buffer for sec in s.sections.values())
    in_buffers = sum(sec.buffer for sec in s.sections.values())
    assert sum(s.delivered.values()) + sum(s.cargo.values()) + in_buffers == produced + initial


def test_energy_is_the_robot_models_energy():
    s = sim()
    s.step(Action(PICKUP, "B"))
    s.step(Action(DELIVER))
    assert s.robot.cumulative_energy_wh > 0
    assert sum(h["energy_wh"] for h in s.history) == pytest.approx(
        s.robot.cumulative_energy_wh, rel=0.01)


def test_distance_matches_the_path_matrix():
    s = sim()
    out = s.step(Action(PICKUP, "A"))
    assert out.distance_m == pytest.approx(s.matrix["charger"]["A"])
    assert s.location == "A"
    assert out.duration_s == pytest.approx(out.distance_m / s.p.speed_m_s + s.p.load_time_s, abs=1)


# ---------------------------------------------------------------------------- actions
def test_pickup_respects_the_payload_limit_and_fills_cargo():
    s = sim("high_demand")
    for _ in range(1200):                       # let the buffers fill
        s._advance(1.0)
    out = s.step(Action(PICKUP, "C"))           # C units are 1.5 kg
    assert out.units == 3 and s.payload_kg == pytest.approx(4.5)
    assert s.units_that_fit("C") == 0           # 5 kg limit reached


def test_deliver_empties_the_robot():
    s = sim()
    s.step(Action(PICKUP, "B"))
    carried = sum(s.cargo.values())
    out = s.step(Action(DELIVER))
    assert out.units == carried and s.payload_kg == 0.0
    assert sum(s.delivered.values()) == carried and s.location == "delivery"


def test_charge_reaches_the_target_and_takes_time():
    s = sim("low_battery_start")
    before = s.time_s
    s.step(Action(CHARGE, value=60.0))
    assert s.robot.battery_percent == pytest.approx(60.0, abs=0.5)
    assert s.time_s - before > 300              # charging is slow on purpose (19 W net)


def test_waiting_at_the_charger_charges_but_elsewhere_drains():
    at_charger = sim("low_battery_start")
    at_charger.step(Action(WAIT, value=600.0))
    away = sim("low_battery_start")
    away.step(Action(PICKUP, "A"))
    battery = away.robot.battery_percent
    away.step(Action(WAIT, value=600.0))
    assert at_charger.robot.battery_percent > 30.0
    assert away.robot.battery_percent < battery


# --------------------------------------------------------------------- legal actions
def test_legal_actions_refuse_pointless_ones():
    s = sim()
    legal = {str(a) for a in s.legal_actions()}
    assert "DELIVER" not in legal               # nothing on board
    assert "WAIT:60" in legal
    s.step(Action(PICKUP, "B"))
    assert "DELIVER" in {str(a) for a in s.legal_actions()}


def test_empty_section_cannot_be_picked_up():
    s = sim()
    assert s.sections["C"].buffer == 0
    assert "PICKUP:C" not in {str(a) for a in s.legal_actions()}


# ------------------------------------------------------------------------- safety
def test_running_out_of_battery_is_a_violation_and_ends_the_shift():
    s = sim(overrides={"robot": {"battery": {"capacity_wh": 0.05, "initial_percent": 5.0}}})
    for _ in range(20):
        if s.done:
            break
        s.step(Action(PICKUP, "B"))
    assert s.done and any(v.startswith("battery_empty") for v in s.summary()["violations"])


def test_working_below_the_reserve_is_recorded():
    s = sim(overrides={"robot": {"battery": {"initial_percent": 16.0, "capacity_wh": 1.0}}})
    s.step(Action(PICKUP, "B"))
    assert any(v.startswith("below_reserve") or v.startswith("battery_empty")
               for v in s.summary()["violations"])


# ------------------------------------------------------------------- reproducibility
def test_same_seed_same_shift_other_seed_differs():
    def shift(seed):
        return run_episode(RuleBasedPolicy(), "fault_burst", seed, CONFIG_DIR)[0]
    assert shift(3) == shift(3)
    assert shift(3) != shift(4)


def test_shift_ends_at_the_configured_length():
    s = sim(shift_duration_s=300.0)
    while not s.done:
        s.step(Action(WAIT, value=60.0))
    assert s.time_s == pytest.approx(300.0, abs=60.0)
    assert s.summary()["shift_s"] <= 361.0


def test_an_episode_is_fast():
    import time
    t0 = time.time()
    run_episode(RuleBasedPolicy(), "balanced", 0, CONFIG_DIR)
    assert time.time() - t0 < 2.0
