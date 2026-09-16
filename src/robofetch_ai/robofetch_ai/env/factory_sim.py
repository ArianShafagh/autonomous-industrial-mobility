"""Fast factory simulator — the same robot and factory, without Gazebo or ROS.

It exists so a decision model can be trained and evaluated over thousands of shifts: one shift
takes well under a second here, against about 30 minutes in Gazebo.

What makes the numbers comparable with the real runs: this file executes the SAME actions
(`mission_plan`), moves the robot with the SAME energy/thermal/wear model (`robot_model`), uses
the SAME maze path lengths (`path_matrix.yaml`) and runs the SAME production sections
(`factory_model`). Only Nav2, the controller and the physics are replaced by "distance / speed".

What it deliberately does not model: navigation failures, localisation error and the exact
driving profile (acceleration, cornering). `nav_overhead_factor` in params.yaml scales path
lengths if Gazebo turns out to drive systematically further than the matrix.
"""
import math
from dataclasses import dataclass, field

from robofetch_core.mission_plan import (CHARGE, CHARGER, DELIVER, DELIVERY, PICKUP, SECTIONS,
                                         WAIT, Action)
from robofetch_core.robot_model import RobotCondition, RobotParams
from robofetch_factory.factory_model import BLOCKED, FAULT, build_sections, load_config
from robofetch_factory.layout import load_path_matrix

LOCATIONS = (CHARGER, DELIVERY) + SECTIONS


@dataclass
class Outcome:
    """What one executed action did."""
    action: Action
    ok: bool
    detail: str = ""
    units: int = 0
    duration_s: float = 0.0
    distance_m: float = 0.0
    energy_wh: float = 0.0
    charged_wh: float = 0.0
    lost_units: float = 0.0        # production lost anywhere while this action ran
    violations: list = field(default_factory=list)


class FactorySim:
    """One shift: robot + three sections + the clock. Deterministic for a given seed."""

    def __init__(self, scenario="balanced", seed=None, config_dir=None, overrides=None,
                 shift_duration_s=None):
        self.cfg = load_config(scenario, config_dir, overrides)
        self.p = RobotParams.from_config(self.cfg)
        self.sim_cfg = self.cfg["mission"]["simulation"]
        self.objective = self.cfg["mission"]["objective"]
        self.matrix = load_path_matrix(config_dir)
        self.seed = self.cfg["time"]["seed"] if seed is None else seed
        self.shift_duration_s = float(shift_duration_s if shift_duration_s is not None
                                      else self.cfg["time"]["shift_duration_s"])
        self.dt = float(self.sim_cfg["step_s"])
        self.reset()

    # ------------------------------------------------------------------------------ state
    def reset(self, seed=None):
        if seed is not None:
            self.seed = seed
        self.sections = build_sections(self.cfg, self.seed)
        self.robot = RobotCondition(self.p)
        self.time_s = 0.0
        self.location = CHARGER
        self.payload_kg = 0.0
        self.cargo = {sid: 0 for sid in self.sections}
        self.delivered = {sid: 0 for sid in self.sections}
        self.distance_m = 0.0
        self.charge_trips = 0
        self.violations = []
        self.history = []
        self.done = False
        self.min_battery = self.robot.battery_percent
        return self.state()

    def state(self):
        """Everything a decision model may look at - the same facts the live robot publishes."""
        sections = {}
        for sid, s in self.sections.items():
            snap = s.snapshot()
            snap["distance_m"] = self.distance_to(sid)
            snap["units_that_fit"] = self.units_that_fit(sid)
            sections[sid] = snap
        return {
            "time_s": self.time_s,
            "time_left_s": max(0.0, self.shift_duration_s - self.time_s),
            "location": self.location,
            "battery_percent": self.robot.battery_percent,
            "temperature_c": self.robot.temperature_c,
            "condition_percent": self.robot.condition_percent,
            "payload_kg": self.payload_kg,
            "cargo_units": sum(self.cargo.values()),
            "free_payload_kg": max(0.0, self.p.max_payload_kg - self.payload_kg),
            "distance_to_delivery_m": self.distance_to(DELIVERY),
            "distance_to_charger_m": self.distance_to(CHARGER),
            "sections": sections,
        }

    def distance_to(self, target, frm=None):
        return (self.matrix[frm or self.location][target]
                * float(self.sim_cfg["nav_overhead_factor"]))

    def charge_targets(self):
        return [float(t) for t in self.sim_cfg["charge_targets_percent"]]

    def units_that_fit(self, sid):
        unit = self.sections[sid].params.unit_mass_kg
        free = max(0.0, self.p.max_payload_kg - self.payload_kg)
        return min(self.sections[sid].buffer, int(math.floor(free / unit + 1e-9)))

    # ------------------------------------------------------------------------ time stepping
    def _advance(self, seconds, distance_m=0.0, motor_load=0.0, docked=False):
        """Run robot and sections forward together; returns (energy drawn, charged, lost units)."""
        before_energy = self.robot.cumulative_energy_wh
        before_charged = self.robot.cumulative_charged_wh
        lost_before = sum(s.lost_units for s in self.sections.values())
        speed = distance_m / seconds if seconds > 0 else 0.0
        left = seconds
        while left > 1e-9 and not self.done:
            step = min(self.dt, left)
            self.robot.step(step, distance_m=speed * step, payload_kg=self.payload_kg,
                            motor_load=motor_load, docked=docked)
            for s in self.sections.values():
                s.step(step)
            self.time_s += step
            left -= step
            self.min_battery = min(self.min_battery, self.robot.battery_percent)
            if self.robot.battery_percent <= 0.0:
                self._violation("battery_empty")
                self.done = True
            if self.robot.temperature_c >= self.p.max_c:
                self._violation("overheated")
            if self.time_s >= self.shift_duration_s:
                self.done = True
        self.distance_m += speed * (seconds - left)
        return (self.robot.cumulative_energy_wh - before_energy,
                self.robot.cumulative_charged_wh - before_charged,
                sum(s.lost_units for s in self.sections.values()) - lost_before)

    def _violation(self, kind):
        if not self.violations or self.violations[-1][0] != kind:
            self.violations.append((kind, round(self.time_s, 1)))

    # ---------------------------------------------------------------------------- actions
    def legal_actions(self):
        """The actions that make sense right now (the mask the AI and the rules share).

        Refused: picking up from a section with nothing the robot can still carry, delivering
        with an empty load, charging while already full at the charger. WAIT is always legal.
        """
        legal = []
        for sid in self.sections:
            if self.units_that_fit(sid) > 0:
                legal.append(Action(PICKUP, sid))
        if sum(self.cargo.values()) > 0:
            legal.append(Action(DELIVER))
        for target in self.charge_targets():
            # Charging to a level the robot is already at would be a no-op.
            if self.robot.battery_percent < target - 1.0:
                legal.append(Action(CHARGE, value=target))
        legal.append(Action(WAIT, value=float(self.sim_cfg["wait_slice_s"])))
        return legal

    def step(self, action):
        """Execute one action to completion. Returns an Outcome."""
        if self.done:
            return Outcome(action, False, "shift over")
        dest = action.destination()
        distance = self.distance_to(dest) if dest else 0.0
        energy = charged = lost = 0.0
        battery_before = self.robot.battery_percent
        t0 = self.time_s
        violations_before = len(self.violations)

        if distance > 0.0:
            e, c, l = self._advance(distance / self.p.speed_m_s, distance_m=distance,
                                    motor_load=self.p.drive_load)
            energy, charged, lost = energy + e, charged + c, lost + l
            self.location = dest
        elif dest:
            self.location = dest

        units, ok, detail = 0, True, ""
        if action.kind == PICKUP:
            e, c, l = self._advance(self.p.load_time_s, motor_load=self.p.dwell_load)
            energy, charged, lost = energy + e, charged + c, lost + l
            section = self.sections[action.target]
            units = section.pickup(max_mass_kg=max(0.0, self.p.max_payload_kg - self.payload_kg))
            self.payload_kg += units * section.params.unit_mass_kg
            self.cargo[action.target] += units
            ok = units > 0
            detail = f"took {units} units"
        elif action.kind == DELIVER:
            e, c, l = self._advance(self.p.unload_time_s, motor_load=self.p.dwell_load)
            energy, charged, lost = energy + e, charged + c, lost + l
            units = sum(self.cargo.values())
            for sid, n in self.cargo.items():
                self.delivered[sid] += n
                self.cargo[sid] = 0
            self.payload_kg = 0.0
            ok = units > 0
            detail = f"delivered {units} units"
        elif action.kind == CHARGE:
            target = min(100.0, action.value)
            self.charge_trips += 1
            while self.robot.battery_percent < target and not self.done:
                e, c, l = self._advance(self.dt, docked=True)
                energy, charged, lost = energy + e, charged + c, lost + l
            detail = f"charged to {self.robot.battery_percent:.1f} %"
        elif action.kind == WAIT:
            docked = self.location == CHARGER
            e, c, l = self._advance(action.value, motor_load=self.p.dwell_load, docked=docked)
            energy, charged, lost = energy + e, charged + c, lost + l
            detail = f"waited {action.value:g} s at {self.location}"

        # Safety: being unable to get home on what is left is a violation, not just bad luck.
        if (self.robot.battery_percent < self.p.reserve_percent and self.location != CHARGER
                and not self.done):
            self._violation("below_reserve")

        outcome = Outcome(action, ok, detail, units, self.time_s - t0, distance, energy, charged,
                          lost, [f"{k}@{t:.0f}s" for k, t in self.violations[violations_before:]])
        self.history.append({"time_s": self.time_s, "action": str(action), "units": units,
                             "battery": round(self.robot.battery_percent, 2),
                             "energy_wh": round(energy, 4), "distance_m": round(distance, 2),
                             "battery_before": round(battery_before, 2)})
        return outcome

    # ---------------------------------------------------------------------------- results
    def reward(self, outcome):
        """Value of one action under the configured objective (also the RL reward)."""
        o = self.objective
        r = outcome.units * float(o["value_per_unit_delivered"]) if outcome.action.kind == DELIVER \
            else 0.0
        r -= outcome.lost_units * float(o["cost_per_lost_unit"])
        r -= outcome.energy_wh * float(o["cost_per_wh"])
        r -= len(outcome.violations) * float(o["cost_per_safety_violation"])
        return r

    def summary(self):
        lost = {sid: s.lost_units for sid, s in self.sections.items()}
        produced = {sid: s.produced_total for sid, s in self.sections.items()}
        blocked = {sid: s.blocked_time_s for sid, s in self.sections.items()}
        delivered = sum(self.delivered.values())
        energy = self.robot.cumulative_energy_wh
        return {
            "scenario": self.cfg["name"], "seed": self.seed,
            "shift_s": round(self.time_s, 1),
            "delivered_units": delivered,
            "delivered_by_section": dict(self.delivered),
            "produced_units": sum(produced.values()),
            "lost_units": round(sum(lost.values()), 2),
            "lost_by_section": {k: round(v, 2) for k, v in lost.items()},
            "blocked_time_s": {k: round(v, 1) for k, v in blocked.items()},
            "energy_wh": round(energy, 3),
            "wh_per_unit": round(energy / delivered, 4) if delivered else None,
            "distance_m": round(self.distance_m, 1),
            "charge_trips": self.charge_trips,
            "min_battery_percent": round(self.min_battery, 1),
            "end_battery_percent": round(self.robot.battery_percent, 1),
            "condition_percent": round(self.robot.condition_percent, 2),
            "violations": [f"{k}@{t:.0f}s" for k, t in self.violations],
            "actions": len(self.history),
            "score": round(self.score(), 2),
        }

    def score(self):
        """The objective over the whole shift (what policies are ranked by)."""
        o = self.objective
        return (sum(self.delivered.values()) * float(o["value_per_unit_delivered"])
                - sum(s.lost_units for s in self.sections.values()) * float(o["cost_per_lost_unit"])
                - self.robot.cumulative_energy_wh * float(o["cost_per_wh"])
                - len(self.violations) * float(o["cost_per_safety_violation"]))


def run_episode(policy, scenario="balanced", seed=0, config_dir=None, overrides=None,
                shift_duration_s=None, trace=False):
    """Run one whole shift with a policy. Returns (summary, decisions)."""
    sim = FactorySim(scenario, seed, config_dir, overrides, shift_duration_s)
    decisions = []
    while not sim.done:
        state = sim.state()
        legal = sim.legal_actions()
        decision = policy.decide(state, legal, sim)
        outcome = sim.step(decision.action)
        if trace:
            decisions.append({"time_s": round(state["time_s"], 1), "action": str(decision.action),
                              "why": decision.explanation, "units": outcome.units,
                              "battery": round(sim.robot.battery_percent, 1),
                              "latency_ms": round(decision.latency_ms, 3),
                              "scores": decision.scores})
    return sim.summary(), decisions
