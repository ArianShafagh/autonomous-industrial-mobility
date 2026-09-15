"""System tests for the robot condition model.

They pin the relationships every decision is built on: carrying costs more, idling still costs
energy, charging refills the pack at the configured power, heat builds up with load and decays,
the battery stays within 0-100 %, and every number really comes from params.yaml.
"""
import os

import pytest

from robofetch_core.robot_model import (RobotCondition, RobotParams, battery_percent_for,
                                        charge_time_s, drive_energy_wh, idle_energy_wh,
                                        simulate_route, trip_energy_wh)
from robofetch_factory.factory_model import load_config

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                          "robofetch_factory", "config")


@pytest.fixture(scope="module")
def cfg():
    return load_config("balanced", CONFIG_DIR)


@pytest.fixture(scope="module")
def p(cfg):
    return RobotParams.from_config(cfg)


# --------------------------------------------------------------------------- config
def test_params_come_from_the_config_file(cfg, p):
    assert p.capacity_wh == cfg["robot"]["battery"]["capacity_wh"]
    assert p.idle_power_w == cfg["robot"]["energy"]["idle_power_w"]
    assert p.speed_m_s == cfg["robot"]["motion"]["speed_m_s"]


def test_missing_parameter_is_reported(cfg):
    broken = {"robot": {k: v for k, v in cfg["robot"].items() if k != "battery"}}
    with pytest.raises(KeyError, match="robot.battery.capacity_wh"):
        RobotParams.from_config(broken)


def test_unused_parameter_is_reported(cfg):
    import copy
    extra = copy.deepcopy(cfg)
    extra["robot"]["energy"]["regen_braking"] = 0.1
    with pytest.raises(KeyError, match="robot.energy.regen_braking"):
        RobotParams.from_config(extra)


def test_scenario_changes_reach_the_model():
    worn = RobotParams.from_config(load_config("worn_robot", CONFIG_DIR))
    low = RobotParams.from_config(load_config("low_battery_start", CONFIG_DIR))
    assert RobotCondition(worn).condition_percent == 45.0
    assert RobotCondition(low).battery_percent == 30.0


# --------------------------------------------------------------------------- energy
def test_heavier_payload_costs_more(p):
    assert drive_energy_wh(p, 10.0, 4.5) > drive_energy_wh(p, 10.0, 0.3)


def test_payload_energy_scales_with_total_mass(p):
    """load_wh_per_m_per_kg = drive_wh_per_m / mass_kg: carrying the robot's own mass again
    doubles drive energy."""
    assert drive_energy_wh(p, 10.0, p.mass_kg) == pytest.approx(2 * drive_energy_wh(p, 10.0, 0.0),
                                                                rel=0.02)


def test_hot_or_worn_drive_costs_more(p):
    base = drive_energy_wh(p, 10.0, 2.0)
    assert drive_energy_wh(p, 10.0, 2.0, temperature_c=60.0) > base
    assert drive_energy_wh(p, 10.0, 2.0, condition_percent=40.0) > base


def test_standing_still_still_costs_energy(p):
    condition = RobotCondition(p)
    condition.step(600.0)
    assert condition.cumulative_energy_wh == pytest.approx(idle_energy_wh(p, 600.0))
    assert condition.battery_percent < 100.0


def test_trip_energy_is_drive_plus_idle_for_the_trip_time(p):
    d = 10.0
    assert trip_energy_wh(p, d, 3.0) == pytest.approx(
        drive_energy_wh(p, d, 3.0) + idle_energy_wh(p, d / p.speed_m_s))


def test_realistic_magnitudes(p):
    """Sanity band for a TurtleBot-class AMR: empty driving 5-40 Wh per km including idle,
    runtime while driving continuously 1-4 h, full charge 30-120 min."""
    per_km = trip_energy_wh(p, 1000.0, 0.0)
    assert 5.0 <= per_km <= 40.0
    runtime_h = p.capacity_wh / (per_km * p.speed_m_s * 3.6)
    assert 1.0 <= runtime_h <= 4.0
    assert 30 * 60 <= charge_time_s(p, 0.0) <= 120 * 60


# ------------------------------------------------------------------------ dynamics
def test_driving_drains_the_battery_and_counts_distance(p):
    condition = RobotCondition(p)
    condition.step(1.0, distance_m=1.0, payload_kg=2.0, motor_load=0.8)
    assert condition.battery_percent < 100.0
    assert condition.cumulative_distance_m == pytest.approx(1.0)


def test_charging_adds_net_charger_power(p):
    condition = RobotCondition(p, battery_percent=50.0)
    condition.step(3600.0, docked=True)
    expected = 50.0 + battery_percent_for(p, (p.charge_power_w - p.idle_power_w))
    assert condition.battery_percent == pytest.approx(min(100.0, expected))
    assert condition.cumulative_energy_wh == 0.0


def test_battery_never_leaves_its_bounds(p):
    empty = RobotCondition(p, battery_percent=0.5)
    empty.step(5.0, distance_m=500.0, payload_kg=5.0, motor_load=1.0)
    assert empty.battery_percent == 0.0
    full = RobotCondition(p, battery_percent=99.9)
    full.step(3600.0, docked=True)
    assert full.battery_percent == 100.0


def test_motor_heats_toward_its_steady_state_and_cools_when_docked(p):
    condition = RobotCondition(p)
    for _ in range(3 * 3600):
        condition.step(1.0, motor_load=1.0)
    steady = p.ambient_c + p.heat_c_per_s / p.cool_per_s
    assert condition.temperature_c == pytest.approx(steady, abs=1.0)
    hot = condition.temperature_c
    condition.step(600.0, docked=True)
    assert condition.temperature_c < hot


def test_overheating_wears_the_drive(p):
    cool = RobotCondition(p)
    hot = RobotCondition(p, temperature_c=p.warn_c + 10.0)
    cool.step(10.0)
    hot.step(10.0)
    assert hot.condition_percent < cool.condition_percent


# ----------------------------------------------------------------------- route simulation
def test_simulate_route_does_not_touch_the_real_condition(p):
    real = RobotCondition(p)
    end, _ = simulate_route(real, [(10.0, 0.0), (10.0, 4.0)], dwell_s=30.0)
    assert real.battery_percent == 100.0
    assert end.battery_percent < 100.0


def test_carrying_only_on_the_loaded_leg_costs_less(p):
    split, _ = simulate_route(RobotCondition(p), [(10.0, 0.0), (10.0, 4.5)])
    whole, _ = simulate_route(RobotCondition(p), [(20.0, 4.5)])
    assert split.cumulative_energy_wh < whole.cumulative_energy_wh


def test_simulated_route_energy_matches_the_closed_form(p):
    end, _ = simulate_route(RobotCondition(p), [(10.0, 0.0), (10.0, 4.0)], dwell_s=30.0)
    expected = (trip_energy_wh(p, 10.0, 0.0) + trip_energy_wh(p, 10.0, 4.0)
                + idle_energy_wh(p, 30.0))
    # small difference only from the temperature penalty building up during the route
    assert end.cumulative_energy_wh == pytest.approx(expected, rel=0.02)
