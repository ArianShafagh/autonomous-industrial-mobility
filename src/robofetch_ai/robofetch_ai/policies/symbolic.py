"""The symbolic half of the neuro-symbolic model: explicit rules, in code, in plain words.

Two kinds of rule, and the difference matters:

* **HARD rules forbid.** They are safety and physics, checked with the robot's own models
  (`robot_model`), and no score from any network can overrule them. This is the guarantee a
  learned policy cannot give by itself: whatever the network prefers, the robot will not strand
  itself, cook its motors, overload itself or work when it needs maintenance.

* **PRIORITY rules rank.** A line that has already stopped outranks one that is merely full-ish;
  a line about to stop outranks a quiet one. The neural scorer then chooses WITHIN the top
  priority tier, which is where judgement (is this trip worth the energy? collect two sections in
  one trip?) actually lives.

Every verdict carries the reasons, so a decision can always be explained.
"""
from dataclasses import dataclass, field

from robofetch_core.mission_plan import CHARGE, CHARGER, DELIVER, PICKUP, WAIT
from robofetch_core.robot_model import RobotCondition, battery_percent_for, simulate_route

TIER_NAMES = {2: "unblock a stopped line", 1: "prevent a line from stopping", 0: "routine"}


@dataclass
class Verdict:
    allowed: bool = True
    tier: int = 0
    refusals: list = field(default_factory=list)   # why a hard rule forbids it
    reasons: list = field(default_factory=list)    # why it is (or is not) urgent

    def why(self):
        return "; ".join(self.refusals if not self.allowed else self.reasons) or "routine"


class SymbolicLayer:
    def __init__(self, cfg):
        ns = cfg["mission"]["neurosymbolic"]
        self.urgency_horizon_s = float(ns["urgency_horizon_s"])
        self.safety_margin = float(ns["safety_margin_percent"])

    # --------------------------------------------------------------------------- hard rules
    def check(self, state, action, p, matrix):
        """Apply every rule to one candidate action."""
        v = Verdict()
        location = state["location"]
        dest = action.destination()

        # H1 maintenance: a worn-out robot may only charge or wait.
        if state["condition_percent"] < p.condition_min_percent and action.kind in (PICKUP, DELIVER):
            v.allowed = False
            v.refusals.append(f"condition {state['condition_percent']:.0f} % is below the "
                              f"maintenance limit ({p.condition_min_percent:.0f} %)")

        # H2 payload: never plan to carry more than the limit.
        if action.kind == PICKUP and state["sections"][action.target]["units_that_fit"] <= 0:
            v.allowed = False
            v.refusals.append(f"nothing at {action.target} that still fits "
                              f"({state['payload_kg']:.1f} of {p.max_payload_kg:.1f} kg carried)")

        # H3 reserve: after this action the robot must still reach the charger with the reserve.
        if dest and action.kind != CHARGE:
            legs = [(matrix[location][dest], state["payload_kg"]),
                    (matrix[dest][CHARGER], state["payload_kg"] if action.kind == PICKUP else 0.0)]
            condition = RobotCondition(p, battery_percent=state["battery_percent"],
                                       temperature_c=state["temperature_c"],
                                       condition_percent=state["condition_percent"])
            end, peak = simulate_route(condition, legs,
                                       dwell_s=p.load_time_s if action.kind == PICKUP
                                       else p.unload_time_s)
            need = p.reserve_percent + self.safety_margin
            if end.battery_percent < need:
                v.allowed = False
                v.refusals.append(
                    f"battery would be {end.battery_percent:.0f} % back at the charger, below the "
                    f"{need:.0f} % reserve")
            # H4 heat: the motors must stay below their limit for the whole route.
            if peak >= p.max_c:
                v.allowed = False
                v.refusals.append(f"motors would reach {peak:.0f} C (limit {p.max_c:.0f} C)")

        if not v.allowed:
            return v

        # ------------------------------------------------------------------- priority rules
        if action.kind == PICKUP:
            section = state["sections"][action.target]
            travel_s = matrix[location][action.target] / p.speed_m_s
            if section["status"] == "BLOCKED":
                v.tier = 2
                v.reasons.append(f"{action.target} is BLOCKED and losing production")
            elif section["time_to_full_s"] <= max(self.urgency_horizon_s, travel_s):
                v.tier = 1
                v.reasons.append(f"{action.target} fills in {section['time_to_full_s']:.0f} s "
                                 f"({travel_s:.0f} s away)")
            else:
                v.reasons.append(f"{action.target} has {section['buffer_units']} units waiting")
        elif action.kind == DELIVER:
            if state["free_payload_kg"] <= 0.01:
                v.tier = 1
                v.reasons.append("the robot is full and cannot collect anything else")
            else:
                v.reasons.append(f"carrying {state['cargo_units']} units")
        elif action.kind == CHARGE:
            home_percent = battery_percent_for(
                p, p.idle_power_w * (matrix[location][CHARGER] / p.speed_m_s) / 3600.0)
            if state["battery_percent"] <= p.reserve_percent + home_percent + self.safety_margin:
                v.tier = 2
                v.reasons.append(f"battery {state['battery_percent']:.0f} % is at the reserve")
            else:
                v.reasons.append(f"top up from {state['battery_percent']:.0f} %")
        elif action.kind == WAIT:
            if state["location"] == CHARGER:
                v.reasons.append("waiting at the charger is free and tops the battery up")
            else:
                v.reasons.append("waiting away from the charger only spends energy")
        return v

    def evaluate(self, state, legal_actions, p, matrix):
        """{action: Verdict} for every legal action."""
        return {action: self.check(state, action, p, matrix) for action in legal_actions}

    @staticmethod
    def allowed_by_tier(verdicts):
        """The allowed actions of the highest priority tier, and that tier."""
        allowed = {a: v for a, v in verdicts.items() if v.allowed}
        if not allowed:
            return {}, None
        top = max(v.tier for v in allowed.values())
        return {a: v for a, v in allowed.items() if v.tier == top}, top
