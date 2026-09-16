"""System tests for the action vocabulary and the cost predictions the executor and AI rely on."""
import os

import pytest

from robofetch_core.mission_plan import (CHARGE, DELIVER, PICKUP, WAIT, Action, parse_action,
                                         parse_plan, predict)
from robofetch_core.robot_model import (RobotParams, battery_percent_for, charge_time_s,
                                        idle_energy_wh, trip_energy_wh)
from robofetch_factory.factory_model import load_config
from robofetch_factory.layout import load_path_matrix

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                          "robofetch_factory", "config")


@pytest.fixture(scope="module")
def p():
    return RobotParams.from_config(load_config("balanced", CONFIG_DIR))


@pytest.fixture(scope="module")
def matrix():
    return load_path_matrix(CONFIG_DIR)


def test_plan_parsing():
    plan = parse_plan(" PICKUP:b ; PICKUP A;DELIVER; CHARGE:90 ;WAIT:60;")
    assert plan == [Action(PICKUP, "B"), Action(PICKUP, "A"), Action(DELIVER),
                    Action(CHARGE, value=90.0), Action(WAIT, value=60.0)]
    assert [str(a) for a in plan] == ["PICKUP:B", "PICKUP:A", "DELIVER", "CHARGE:90", "WAIT:60"]
    assert parse_action("CHARGE") == Action(CHARGE, value=100.0)


@pytest.mark.parametrize("bad", ["PICKUP", "PICKUP:D", "DELIVER:A", "CHARGE:0", "CHARGE:120",
                                 "WAIT", "WAIT:-5", "FLY:A", ""])
def test_invalid_actions_are_rejected(bad):
    with pytest.raises(ValueError):
        parse_action(bad)


def test_destinations():
    assert Action(PICKUP, "C").destination() == "C"
    assert Action(DELIVER).destination() == "delivery"
    assert Action(CHARGE, value=80).destination() == "charger"
    assert Action(WAIT, value=10).destination() is None


def test_pickup_prediction_uses_maze_distance_and_load_time(p, matrix):
    pred = predict(p, matrix, Action(PICKUP, "B"), "charger", 100.0, 0.0)
    assert pred.distance_m == matrix["charger"]["B"]
    assert pred.duration_s == pytest.approx(matrix["charger"]["B"] / p.speed_m_s + p.load_time_s)
    expected = trip_energy_wh(p, matrix["charger"]["B"], 0.0) + idle_energy_wh(p, p.load_time_s)
    assert pred.energy_wh == pytest.approx(expected)
    assert pred.battery_end_percent == pytest.approx(100.0 - battery_percent_for(p, expected))


def test_loaded_delivery_costs_more_than_empty(p, matrix):
    empty = predict(p, matrix, Action(DELIVER), "A", 90.0, 0.0)
    loaded = predict(p, matrix, Action(DELIVER), "A", 90.0, 4.0)
    assert loaded.energy_wh > empty.energy_wh
    assert loaded.duration_s == empty.duration_s


def test_charge_prediction_reaches_target(p, matrix):
    pred = predict(p, matrix, Action(CHARGE, value=90.0), "delivery", 50.0, 0.0)
    arrive = 50.0 - battery_percent_for(p, trip_energy_wh(p, matrix["delivery"]["charger"], 0.0))
    assert pred.battery_end_percent == pytest.approx(90.0)
    assert pred.duration_s == pytest.approx(
        matrix["delivery"]["charger"] / p.speed_m_s + charge_time_s(p, arrive, 90.0))


def test_waiting_at_the_charger_charges_elsewhere_drains(p, matrix):
    at_charger = predict(p, matrix, Action(WAIT, value=600.0), "charger", 50.0, 0.0)
    elsewhere = predict(p, matrix, Action(WAIT, value=600.0), "A", 50.0, 0.0)
    assert at_charger.battery_end_percent > 50.0
    assert elsewhere.battery_end_percent < 50.0
    assert at_charger.distance_m == elsewhere.distance_m == 0.0
