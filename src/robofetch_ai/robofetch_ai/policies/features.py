"""The facts a decision is made from, as numbers — one vector per CANDIDATE ACTION.

Scoring each action separately (rather than one vector for the whole state) is what lets the
neural part stay small and generalise: it learns "is this action worth it, given what it costs
and what it saves", not "what should I do in this exact factory".

The same function is used when collecting training data and when deciding live, so a feature can
never mean two different things.
"""
from robofetch_core.mission_plan import CHARGE, CHARGER, DELIVER, DELIVERY, PICKUP, WAIT
from robofetch_core.robot_model import battery_percent_for, trip_energy_wh

FEATURE_NAMES = [
    "is_pickup", "is_deliver", "is_charge", "is_wait",
    "battery", "temperature", "condition", "payload_full", "cargo_units", "time_left",
    "distance", "energy_cost_percent", "battery_after", "battery_after_home",
    "target_fill", "target_units_gain", "target_mass_gain", "target_time_to_full",
    "target_rate", "target_health", "target_blocked", "target_fault",
    "worst_other_fill", "worst_other_time_to_full", "any_other_blocked",
    "charge_target", "charge_gain",
]


def _time_scale(seconds, horizon=600.0):
    """Bounded 0..1: 0 = right now, 1 = far away (or never)."""
    if seconds is None or seconds != seconds:          # NaN
        return 1.0
    if seconds == float("inf"):
        return 1.0
    return max(0.0, min(1.0, seconds / horizon))


def symbolic_estimate(state, action, p, matrix, objective):
    """The obvious part of an action's value, computed from the models - no learning involved.

    Units the action moves, valued with the objective, minus the energy it costs. The neural
    scorer then only has to learn the CORRECTION to this (what happens later: which line blocks
    next, whether the battery runs out, whether a fuller load would have been better). Starting
    from a sensible estimate is what keeps the model safe when the network is unsure.
    """
    dest = action.destination()
    distance = matrix[state["location"]][dest] if dest else 0.0
    seconds = distance / p.speed_m_s
    if action.kind == PICKUP:
        seconds += p.load_time_s
        units = state["sections"][action.target]["units_that_fit"]
    elif action.kind == DELIVER:
        seconds += p.unload_time_s
        units = state["cargo_units"]
    elif action.kind == WAIT:
        seconds += action.value
        units = 0
    else:                                   # CHARGE: no units moved, and charging is not "spent"
        units = 0
    energy = trip_energy_wh(p, distance, state["payload_kg"], state["temperature_c"],
                            state["condition_percent"])
    if action.kind != CHARGE:
        energy += p.idle_power_w * seconds / 3600.0
    return (units * float(objective["value_per_unit_delivered"])
            - energy * float(objective["cost_per_wh"]))


def action_features(state, action, p, matrix):
    """Feature vector for one candidate action in one state (list of floats, FEATURE_NAMES)."""
    location = state["location"]
    sections = state["sections"]
    dest = action.destination()
    distance = matrix[location][dest] if dest else 0.0
    energy = trip_energy_wh(p, distance, state["payload_kg"], state["temperature_c"],
                            state["condition_percent"])
    cost_percent = battery_percent_for(p, energy)
    battery_after = state["battery_percent"] - cost_percent
    home = matrix[dest][CHARGER] if dest else matrix[location][CHARGER]
    home_percent = battery_percent_for(
        p, trip_energy_wh(p, home, state["payload_kg"], state["temperature_c"],
                          state["condition_percent"]))

    if action.kind == PICKUP:
        target = sections[action.target]
        gain_units = target["units_that_fit"]
        gain_mass = gain_units * target["unit_mass_kg"]
        fill, ttf = target["buffer_fill"], target["time_to_full_s"]
        rate = target["rate_actual_per_hour"] / max(1.0, target["rate_nominal_per_hour"])
        health = target["health_percent"] / 100.0
        blocked = 1.0 if target["status"] == "BLOCKED" else 0.0
        fault = 1.0 if target["status"] == "FAULT" else 0.0
        others = [s for sid, s in sections.items() if sid != action.target]
    else:
        gain_units = gain_mass = fill = rate = health = blocked = fault = 0.0
        ttf = float("inf")
        others = list(sections.values())

    worst_fill = max((s["buffer_fill"] for s in others), default=0.0)
    worst_ttf = min((s["time_to_full_s"] for s in others), default=float("inf"))
    any_blocked = 1.0 if any(s["status"] == "BLOCKED" for s in others) else 0.0

    return [
        1.0 if action.kind == PICKUP else 0.0,
        1.0 if action.kind == DELIVER else 0.0,
        1.0 if action.kind == CHARGE else 0.0,
        1.0 if action.kind == WAIT else 0.0,
        state["battery_percent"] / 100.0,
        (state["temperature_c"] - p.ambient_c) / max(1.0, p.max_c - p.ambient_c),
        state["condition_percent"] / 100.0,
        state["payload_kg"] / p.max_payload_kg,
        min(1.0, state["cargo_units"] / 20.0),
        _time_scale(state["time_left_s"], 3600.0),
        min(1.0, distance / 25.0),
        min(1.0, cost_percent / 20.0),
        battery_after / 100.0,
        (battery_after - home_percent) / 100.0,
        fill,
        min(1.0, gain_units / 15.0),
        gain_mass / p.max_payload_kg,
        _time_scale(ttf),
        rate,
        health,
        blocked,
        fault,
        worst_fill,
        _time_scale(worst_ttf),
        any_blocked,
        action.value / 100.0 if action.kind == CHARGE else 0.0,
        max(0.0, action.value - state["battery_percent"]) / 100.0 if action.kind == CHARGE else 0.0,
    ]
