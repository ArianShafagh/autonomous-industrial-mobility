"""Robot actions and their predicted cost — pure Python, no ROS.

The action vocabulary is shared by everything that decides or executes: the scripted plans used
for testing, the fast simulator, the AI models and the live mission executor.

    PICKUP <section>   drive to A/B/C, load for load_time_s, take as many units as still fit
    DELIVER            drive to the delivery point, unload for unload_time_s
    CHARGE <percent>   drive to the charger and charge until the battery reaches <percent>
    WAIT <seconds>     stay where the robot is; waiting AT the charger means charging

`predict()` returns what an action should cost from the robot's current state using the same
robot_model equations and the generated maze path lengths, so predicted and measured values can
be compared for every executed action.
"""
from dataclasses import dataclass, field

from robofetch_core.robot_model import (battery_percent_for, charge_time_s, drive_energy_wh,
                                        idle_energy_wh, net_charge_power_w, travel_time_s)

PICKUP, DELIVER, CHARGE, WAIT = "PICKUP", "DELIVER", "CHARGE", "WAIT"
SECTIONS = ("A", "B", "C")
DELIVERY, CHARGER = "delivery", "charger"


@dataclass(frozen=True)
class Action:
    kind: str
    target: str = ""          # section for PICKUP
    value: float = 0.0        # battery % for CHARGE, seconds for WAIT

    def destination(self):
        return {PICKUP: self.target, DELIVER: DELIVERY, CHARGE: CHARGER}.get(self.kind)

    def __str__(self):
        if self.kind == PICKUP:
            return f"PICKUP:{self.target}"
        if self.kind in (CHARGE, WAIT):
            return f"{self.kind}:{self.value:g}"
        return self.kind


def parse_action(text):
    parts = [p.strip() for p in text.strip().replace(" ", ":", 1).split(":") if p.strip()]
    if not parts:
        raise ValueError("empty action")
    kind = parts[0].upper()
    if kind == PICKUP:
        if len(parts) != 2 or parts[1].upper() not in SECTIONS:
            raise ValueError(f"PICKUP needs a section {SECTIONS}: '{text}'")
        return Action(PICKUP, parts[1].upper())
    if kind == DELIVER:
        if len(parts) != 1:
            raise ValueError(f"DELIVER takes no argument: '{text}'")
        return Action(DELIVER)
    if kind == CHARGE:
        target = float(parts[1]) if len(parts) > 1 else 100.0
        if not 0.0 < target <= 100.0:
            raise ValueError(f"CHARGE target must be in (0, 100]: '{text}'")
        return Action(CHARGE, value=target)
    if kind == WAIT:
        if len(parts) != 2 or float(parts[1]) <= 0.0:
            raise ValueError(f"WAIT needs a positive number of seconds: '{text}'")
        return Action(WAIT, value=float(parts[1]))
    raise ValueError(f"unknown action '{text}' (PICKUP, DELIVER, CHARGE, WAIT)")


def parse_plan(text):
    """'PICKUP:B; PICKUP:A; DELIVER; CHARGE:90; WAIT:60' -> [Action, ...]"""
    return [parse_action(item) for item in text.split(";") if item.strip()]


@dataclass
class Prediction:
    distance_m: float
    duration_s: float
    energy_wh: float          # drawn from the battery
    charged_wh: float = 0.0
    battery_end_percent: float = 0.0
    notes: list = field(default_factory=list)


def predict(p, matrix, action, location, battery_percent, payload_kg,
            temperature_c=25.0, condition_percent=100.0):
    """Expected cost of `action` for a robot at `location` (a POI name)."""
    dest = action.destination()
    distance = matrix[location][dest] if dest else 0.0
    drive_s = travel_time_s(p, distance)
    energy = (drive_energy_wh(p, distance, payload_kg, temperature_c, condition_percent)
              + idle_energy_wh(p, drive_s))
    battery = battery_percent - battery_percent_for(p, energy)
    duration = drive_s
    charged = 0.0

    if action.kind == PICKUP:
        duration += p.load_time_s
        energy += idle_energy_wh(p, p.load_time_s)
        battery -= battery_percent_for(p, idle_energy_wh(p, p.load_time_s))
    elif action.kind == DELIVER:
        duration += p.unload_time_s
        energy += idle_energy_wh(p, p.unload_time_s)
        battery -= battery_percent_for(p, idle_energy_wh(p, p.unload_time_s))
    elif action.kind == CHARGE:
        target = max(battery, action.value)
        duration += charge_time_s(p, battery, target)
        charged = (target - battery) / 100.0 * p.capacity_wh
        battery = target
    elif action.kind == WAIT:
        duration += action.value
        if location == CHARGER:
            charged = min(net_charge_power_w(p) * action.value / 3600.0,
                          (100.0 - battery) / 100.0 * p.capacity_wh)
            battery += battery_percent_for(p, charged)
        else:
            idle = idle_energy_wh(p, action.value)
            energy += idle
            battery -= battery_percent_for(p, idle)
    return Prediction(distance, duration, energy, charged, max(0.0, battery))
