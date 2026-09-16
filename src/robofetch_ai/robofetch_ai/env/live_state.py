"""The live robot's situation, in the shape the policies already understand.

The decision models were written against `FactorySim`. This class presents the REAL system - the
section status messages and the robot telemetry coming from ROS - through exactly the same
interface (`state()`, `legal_actions()`, `p`, `matrix`, ...), so the code that decides in the fast
simulator is literally the same code that decides on the robot. No second implementation, no
chance of the two drifting apart.

It holds no ROS types: the caller passes plain numbers, which keeps this importable anywhere
(the decision service runs outside ROS).
"""
import math

from robofetch_core.mission_plan import CHARGE, CHARGER, DELIVER, DELIVERY, PICKUP, WAIT, Action
from robofetch_core.robot_model import RobotParams
from robofetch_factory.factory_model import load_config
from robofetch_factory.layout import load_path_matrix


class LiveState:
    """Build a decision-ready state from what the live system reports."""

    def __init__(self, scenario="balanced", config_dir=None):
        self.cfg = load_config(scenario, config_dir)
        self.p = RobotParams.from_config(self.cfg)
        self.matrix = load_path_matrix(config_dir)
        self.sim_cfg = self.cfg["mission"]["simulation"]
        self.objective = self.cfg["mission"]["objective"]
        self.shift_duration_s = float(self.cfg["time"]["shift_duration_s"])
        self._state = None

    # ------------------------------------------------------------------------------ input
    def update(self, *, time_s, location, battery_percent, temperature_c, condition_percent,
               payload_kg, cargo_units, sections, shift_duration_s=None):
        """`sections` maps section id -> the fields of a SectionStatus message (as a dict)."""
        if shift_duration_s is not None:
            self.shift_duration_s = float(shift_duration_s)
        free_kg = max(0.0, self.p.max_payload_kg - payload_kg)
        prepared = {}
        for sid, raw in sections.items():
            section = dict(raw)
            unit_mass = float(section.get("unit_mass_kg", 0.0)) or 1e-9
            section.setdefault("status", section.get("status_text", "RUNNING"))
            section["distance_m"] = self.matrix[location][sid]
            section["units_that_fit"] = min(int(section.get("buffer_units", 0)),
                                            int(math.floor(free_kg / unit_mass + 1e-9)))
            if not math.isfinite(section.get("time_to_full_s", math.inf)):
                section["time_to_full_s"] = math.inf
            prepared[sid] = section
        self._state = {
            "time_s": float(time_s),
            "time_left_s": max(0.0, self.shift_duration_s - float(time_s)),
            "location": location,
            "battery_percent": float(battery_percent),
            "temperature_c": float(temperature_c),
            "condition_percent": float(condition_percent),
            "payload_kg": float(payload_kg),
            "cargo_units": int(cargo_units),
            "free_payload_kg": free_kg,
            "distance_to_delivery_m": self.matrix[location][DELIVERY],
            "distance_to_charger_m": self.matrix[location][CHARGER],
            "sections": prepared,
        }
        return self._state

    # --------------------------------------------------------- the FactorySim interface
    def state(self):
        if self._state is None:
            raise RuntimeError("LiveState.update() must be called before a decision")
        return self._state

    def charge_targets(self):
        return [float(t) for t in self.sim_cfg["charge_targets_percent"]]

    def legal_actions(self):
        """The same rule as in the fast simulator, applied to the live numbers."""
        s = self.state()
        legal = [Action(PICKUP, sid) for sid, sec in s["sections"].items()
                 if sec["units_that_fit"] > 0]
        if s["cargo_units"] > 0:
            legal.append(Action(DELIVER))
        for target in self.charge_targets():
            if s["battery_percent"] < target - 1.0:
                legal.append(Action(CHARGE, value=target))
        legal.append(Action(WAIT, value=float(self.sim_cfg["wait_slice_s"])))
        return legal

    # PPO reads these through the Gymnasium wrapper.
    @property
    def sections(self):
        return self.state()["sections"]

    @property
    def robot(self):
        class _Robot:
            battery_percent = self.state()["battery_percent"]
            temperature_c = self.state()["temperature_c"]
            condition_percent = self.state()["condition_percent"]
        return _Robot()
