"""What the robot does when the decision service does not answer.

Deliberately tiny, dependency-free and dull: the robot must keep working safely even with no AI
at all, and this must not itself be able to fail. It lives in robofetch_core (not robofetch_ai)
precisely so that the fallback does not depend on the thing that failed.

    1. if the battery no longer comfortably covers getting home -> charge
    2. else serve the section that is blocked, or fills soonest, among those with units that fit
    3. else deliver whatever is on board
    4. else wait
"""
from robofetch_core.mission_plan import CHARGE, CHARGER, DELIVER, PICKUP, WAIT, Action
from robofetch_core.robot_model import battery_percent_for, trip_energy_wh


def decide(state, p, charge_targets, wait_slice_s=60.0):
    """Returns (action, reason). `state` has the same shape the decision service receives."""
    battery = state["battery_percent"]
    home = battery_percent_for(
        p, trip_energy_wh(p, state["distance_to_charger_m"], state["payload_kg"],
                          state["temperature_c"], state["condition_percent"]))
    if battery <= home + p.reserve_percent + 2.0:
        target = max(t for t in charge_targets) if charge_targets else 100.0
        return Action(CHARGE, value=target), (
            f"fallback: battery {battery:.0f} % barely covers getting home, charging to {target:.0f} %")

    candidates = [(sid, sec) for sid, sec in state["sections"].items()
                  if sec.get("units_that_fit", 0) > 0]
    if candidates:
        def urgency(item):
            _, sec = item
            blocked = sec.get("status", "") == "BLOCKED"
            return (1 if blocked else 0, -sec.get("time_to_full_s", float("inf")))
        sid, sec = max(candidates, key=urgency)
        why = "is BLOCKED" if sec.get("status") == "BLOCKED" else \
            f"fills in {sec.get('time_to_full_s', float('inf')):.0f} s"
        return Action(PICKUP, sid), f"fallback: section {sid} {why}"

    if state["cargo_units"] > 0:
        return Action(DELIVER), f"fallback: carrying {state['cargo_units']} units, nothing to collect"
    return Action(WAIT, value=wait_slice_s), "fallback: nothing to do"
