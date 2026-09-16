"""Rule-based reference policy — the line every learned model must beat.

Deliberately simple and completely explainable, built only from what an operator would say:

    1. keep the robot able to get home         charge when the battery would not cover
                                               "reach the charger + reserve"
    2. never let a line stand still            serve BLOCKED sections first, then the one that
                                               will block soonest
    3. do not drive half empty                 fill the load on the way if another section is
                                               close and has units
    4. deliver what you carry                  when full, or when nothing is worth collecting
    5. wait at the charger, not in the aisle   waiting anywhere else only wastes energy

It is also the fallback used by the live decision service when the AI model is unavailable, so
the robot always has a sane behaviour to fall back to.
"""
from robofetch_core.mission_plan import (CHARGE, CHARGER, DELIVER, DELIVERY, PICKUP, WAIT, Action)
from robofetch_core.robot_model import battery_percent_for, trip_energy_wh

from robofetch_ai.policies.base import Decision, Policy


class RuleBasedPolicy(Policy):
    name = "rule"

    def __init__(self, urgency_horizon_s=180.0, fill_threshold=0.35):
        # A section that will block within this much time counts as urgent.
        self.urgency_horizon_s = urgency_horizon_s
        # Below this buffer fill a trip is not worth making unless the line is about to block.
        self.fill_threshold = fill_threshold

    def battery_needed(self, p, state, action):
        """Battery % this action costs, plus getting home afterwards, plus the reserve."""
        sim_matrix_from = action.destination() or state["location"]
        to_goal = state["sections"][action.target]["distance_m"] if action.kind == PICKUP else (
            state["distance_to_delivery_m"] if action.kind == DELIVER else 0.0)
        energy = trip_energy_wh(p, to_goal, state["payload_kg"], state["temperature_c"],
                               state["condition_percent"])
        home = trip_energy_wh(p, self._home_distance(state, sim_matrix_from), state["payload_kg"],
                              state["temperature_c"], state["condition_percent"])
        return battery_percent_for(p, energy + home) + p.reserve_percent

    @staticmethod
    def _home_distance(state, location):
        # Worst case from the goal: the longest way home among the known distances.
        return max(state["distance_to_charger_m"], state["distance_to_delivery_m"])

    def decide(self, state, legal_actions, sim=None):
        p = sim.p
        kinds = {a.kind: a for a in legal_actions}
        pickups = [a for a in legal_actions if a.kind == PICKUP]
        battery = state["battery_percent"]

        # --- rule 1: be able to get home -------------------------------------------------
        if CHARGE in kinds:
            reserve_needed = battery_percent_for(
                p, trip_energy_wh(p, state["distance_to_charger_m"], state["payload_kg"],
                                  state["temperature_c"], state["condition_percent"]))
            if battery <= reserve_needed + p.reserve_percent:
                if kinds.get(DELIVER) and state["distance_to_delivery_m"] <= \
                        state["distance_to_charger_m"]:
                    return Decision(kinds[DELIVER],
                                    f"battery {battery:.0f} % is near the reserve and the delivery "
                                    "point is on the way to the charger")
                return Decision(kinds[CHARGE],
                                f"battery {battery:.0f} % only just covers getting home "
                                f"({reserve_needed + p.reserve_percent:.0f} % needed)")

        # --- rule 2/3: who needs the robot most ------------------------------------------
        def urgency(action):
            s = state["sections"][action.target]
            blocked = s["status"] == "BLOCKED"
            soon = s["time_to_full_s"] <= self.urgency_horizon_s
            return (2 if blocked else 1 if soon else 0,
                    s["units_that_fit"] * s["unit_mass_kg"] / max(1.0, s["distance_m"]))

        worth = [a for a in pickups
                 if state["sections"][a.target]["buffer_fill"] >= self.fill_threshold
                 or state["sections"][a.target]["status"] == "BLOCKED"
                 or state["sections"][a.target]["time_to_full_s"] <= self.urgency_horizon_s]
        if worth:
            best = max(worth, key=urgency)
            s = state["sections"][best.target]
            why = (f"section {best.target} is BLOCKED" if s["status"] == "BLOCKED" else
                   f"section {best.target} fills in {s['time_to_full_s']:.0f} s"
                   if s["time_to_full_s"] <= self.urgency_horizon_s else
                   f"section {best.target} has the best load per metre")
            return Decision(best, f"{why} ({s['buffer_units']}/{s['buffer_capacity']} units, "
                                  f"{s['distance_m']:.1f} m away)")

        # --- rule 4: deliver what we carry ------------------------------------------------
        if DELIVER in kinds:
            return Decision(kinds[DELIVER],
                            f"carrying {state['cargo_units']} units and nothing is urgent")

        # --- rule 5: wait where waiting is free -------------------------------------------
        if state["location"] != CHARGER and CHARGE in kinds:
            return Decision(kinds[CHARGE], "nothing to do - waiting at the charger is free "
                                           "(and tops the battery up)")
        return Decision(kinds[WAIT], "nothing ready to collect; waiting at the charger")
