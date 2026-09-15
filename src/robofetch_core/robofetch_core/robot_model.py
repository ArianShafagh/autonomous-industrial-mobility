"""Robot condition model — battery, energy, temperature and wear. Pure Python, no ROS.

Every number comes from the `robot:` section of robofetch_factory/config/params.yaml through
`RobotParams.from_config(cfg)`; this file holds equations only. The same code runs inside the
live robot_state_node (Gazebo) and the fast simulator used to train and evaluate the AI, so an
energy figure means the same thing in both.

Four quantities evolve together and feed back into each other:

    energy      idle electronics power over time, plus drive energy per metre that grows with
                the payload mass, motor temperature and drive wear
    battery     drained by that energy; recharged at the charger (charger also powers the robot)
    temperature rises with motor load and payload, decays toward ambient
    condition   degrades with energy throughput and with time spent overheated
"""
from dataclasses import asdict, dataclass, fields

# Map from params.yaml (robot.<group>.<key>) to the flat RobotParams field names.
_CONFIG_MAP = {
    "mass_kg": ("mass_kg",),
    "capacity_wh": ("battery", "capacity_wh"),
    "reserve_percent": ("battery", "reserve_percent"),
    "charge_power_w": ("battery", "charge_power_w"),
    "initial_battery_percent": ("battery", "initial_percent"),
    "idle_power_w": ("energy", "idle_power_w"),
    "drive_wh_per_m": ("energy", "drive_wh_per_m"),
    "load_wh_per_m_per_kg": ("energy", "load_wh_per_m_per_kg"),
    "temp_loss_per_c": ("energy", "temp_loss_per_c"),
    "worn_extra_draw": ("energy", "worn_extra_draw"),
    "ambient_c": ("thermal", "ambient_c"),
    "heat_c_per_s": ("thermal", "heat_c_per_s"),
    "cool_per_s": ("thermal", "cool_per_s"),
    "payload_heat_per_kg": ("thermal", "payload_heat_per_kg"),
    "dock_cooling_factor": ("thermal", "dock_cooling_factor"),
    "warn_c": ("thermal", "warn_c"),
    "max_c": ("thermal", "max_c"),
    "resume_c": ("thermal", "resume_c"),
    "condition_min_percent": ("wear", "condition_min_percent"),
    "wear_per_wh": ("wear", "per_wh"),
    "wear_per_c_s_above_warn": ("wear", "per_c_s_above_warn"),
    "initial_condition_percent": ("wear", "initial_percent"),
    "speed_m_s": ("motion", "speed_m_s"),
    "drive_load": ("motion", "drive_load"),
    "dwell_load": ("motion", "dwell_load"),
    "max_payload_kg": ("handling", "max_payload_kg"),
    "load_time_s": ("handling", "load_time_s"),
    "unload_time_s": ("handling", "unload_time_s"),
}

T_REF_C = 25.0     # temperature at which drive efficiency is nominal (definition, not a tunable)


@dataclass(frozen=True)
class RobotParams:
    """All robot numbers. No defaults: they must come from params.yaml."""
    mass_kg: float
    capacity_wh: float
    reserve_percent: float
    charge_power_w: float
    initial_battery_percent: float
    idle_power_w: float
    drive_wh_per_m: float
    load_wh_per_m_per_kg: float
    temp_loss_per_c: float
    worn_extra_draw: float
    ambient_c: float
    heat_c_per_s: float
    cool_per_s: float
    payload_heat_per_kg: float
    dock_cooling_factor: float
    warn_c: float
    max_c: float
    resume_c: float
    condition_min_percent: float
    wear_per_wh: float
    wear_per_c_s_above_warn: float
    initial_condition_percent: float
    speed_m_s: float
    drive_load: float
    dwell_load: float
    max_payload_kg: float
    load_time_s: float
    unload_time_s: float

    @classmethod
    def from_config(cls, cfg):
        """Build from the full config dict (as returned by load_config) or its `robot` part."""
        robot = cfg.get("robot", cfg)

        def leaves(node, prefix=()):
            for key, value in node.items():
                if isinstance(value, dict):
                    yield from leaves(value, prefix + (key,))
                else:
                    yield prefix + (key,)
        unused = set(leaves(robot)) - set(_CONFIG_MAP.values())
        if unused:
            raise KeyError("params.yaml has robot parameters no code reads: "
                           + ", ".join("robot." + ".".join(u) for u in sorted(unused)))
        values = {}
        for name, path in _CONFIG_MAP.items():
            node = robot
            for key in path:
                if not isinstance(node, dict) or key not in node:
                    raise KeyError(f"params.yaml is missing robot.{'.'.join(path)}")
                node = node[key]
            values[name] = float(node)
        return cls(**values)

    def replace(self, **changes):
        data = asdict(self)
        unknown = set(changes) - {f.name for f in fields(self)}
        if unknown:
            raise KeyError(f"unknown RobotParams field(s): {sorted(unknown)}")
        data.update(changes)
        return RobotParams(**data)


# ----------------------------------------------------------------------------- equations
def efficiency_penalty(p, temperature_c, condition_percent):
    """Multiplier (>= 1) on drive energy from heat and wear. Cold and new is the best case."""
    thermal = 1.0 + p.temp_loss_per_c * max(0.0, temperature_c - T_REF_C)
    health = 1.0 + p.worn_extra_draw * max(0.0, 1.0 - condition_percent / 100.0)
    return thermal * health


def drive_energy_wh(p, distance_m, payload_kg, temperature_c=T_REF_C, condition_percent=100.0):
    """Motor energy to move `distance_m` carrying `payload_kg` (excludes idle electronics)."""
    if distance_m <= 0.0:
        return 0.0
    per_m = p.drive_wh_per_m + p.load_wh_per_m_per_kg * max(0.0, payload_kg)
    return distance_m * per_m * efficiency_penalty(p, temperature_c, condition_percent)


def idle_energy_wh(p, seconds):
    """Electronics energy (computer, lidar, drivers) over a period, moving or not."""
    return p.idle_power_w * max(0.0, seconds) / 3600.0


def travel_time_s(p, distance_m):
    return max(0.0, distance_m) / p.speed_m_s


def trip_energy_wh(p, distance_m, payload_kg, temperature_c=T_REF_C, condition_percent=100.0):
    """Total energy of one driven leg: drive energy plus idle power for its duration."""
    return (drive_energy_wh(p, distance_m, payload_kg, temperature_c, condition_percent)
            + idle_energy_wh(p, travel_time_s(p, distance_m)))


def battery_percent_for(p, energy_wh):
    return 100.0 * energy_wh / p.capacity_wh


def net_charge_power_w(p):
    """Power actually going into the pack at the charger (the charger also runs the robot)."""
    return p.charge_power_w - p.idle_power_w


def charge_time_s(p, from_percent, to_percent=100.0):
    need_wh = max(0.0, to_percent - from_percent) / 100.0 * p.capacity_wh
    power = net_charge_power_w(p)
    return float("inf") if power <= 0.0 else need_wh / power * 3600.0


# ----------------------------------------------------------------------------- state
@dataclass
class RobotCondition:
    """The robot's physical condition, integrated forward one tick at a time."""

    params: RobotParams
    battery_percent: float = None
    temperature_c: float = None
    condition_percent: float = None
    cumulative_distance_m: float = 0.0
    cumulative_energy_wh: float = 0.0       # energy drawn from the battery (drive + idle)
    cumulative_charged_wh: float = 0.0      # energy put back at the charger
    uptime_s: float = 0.0
    overheat_s: float = 0.0

    def __post_init__(self):
        p = self.params
        if self.battery_percent is None:
            self.battery_percent = p.initial_battery_percent
        if self.temperature_c is None:
            self.temperature_c = p.ambient_c
        if self.condition_percent is None:
            self.condition_percent = p.initial_condition_percent

    def copy(self):
        data = asdict(self)
        data["params"] = self.params
        return RobotCondition(**data)

    def step(self, dt, distance_m=0.0, payload_kg=0.0, motor_load=0.0, docked=False):
        """Advance by `dt` seconds. `motor_load` 0..1; `docked` = on the charger, drive off."""
        if dt <= 0.0:
            return self
        p = self.params
        self.uptime_s += dt

        if docked:
            added = net_charge_power_w(p) * dt / 3600.0
            room = (100.0 - self.battery_percent) / 100.0 * p.capacity_wh
            added = max(0.0, min(added, room))
            self.battery_percent += battery_percent_for(p, added)
            self.cumulative_charged_wh += added
            used = 0.0
        else:
            used = (drive_energy_wh(p, distance_m, payload_kg, self.temperature_c,
                                    self.condition_percent) + idle_energy_wh(p, dt))
            self.battery_percent = max(0.0, self.battery_percent - battery_percent_for(p, used))
            self.cumulative_energy_wh += used
            self.cumulative_distance_m += max(0.0, distance_m)
        self.battery_percent = min(100.0, self.battery_percent)

        cooling = p.cool_per_s * (p.dock_cooling_factor if docked else 1.0)
        heating = 0.0 if docked else p.heat_c_per_s * motor_load * (
            1.0 + p.payload_heat_per_kg * max(0.0, payload_kg))
        self.temperature_c += dt * (heating - cooling * (self.temperature_c - p.ambient_c))
        self.temperature_c = max(p.ambient_c, self.temperature_c)

        overheat = max(0.0, self.temperature_c - p.warn_c)
        if overheat > 0.0:
            self.overheat_s += dt
        self.condition_percent = max(0.0, self.condition_percent - (
            p.wear_per_wh * used + p.wear_per_c_s_above_warn * overheat * dt))
        return self

    def as_dict(self):
        data = asdict(self)
        data.pop("params")
        return data


def simulate_route(condition, legs, dt=1.0, dwell_s=0.0):
    """Forward-simulate driven legs plus one stationary dwell on a COPY of `condition`.

    `legs` is an iterable of (distance_m, payload_kg): e.g. the empty leg to a section followed
    by the loaded leg to the delivery point, so payload energy and heat are only charged where
    the payload is carried. `dwell_s` (loading/unloading) is simulated once, after the legs.
    Temperature is a trajectory, so the PEAK is returned as well as the end state.

    Returns (condition_at_the_end, peak_temperature_c).
    """
    p = condition.params
    sim = condition.copy()
    peak = sim.temperature_c

    def walk(total_s, distance_m, payload_kg, load):
        nonlocal peak
        speed = distance_m / total_s if total_s > 0.0 else 0.0
        elapsed = 0.0
        while elapsed < total_s:
            step = min(dt, total_s - elapsed)
            sim.step(step, distance_m=speed * step, payload_kg=payload_kg, motor_load=load)
            peak = max(peak, sim.temperature_c)
            elapsed += step

    for distance_m, payload_kg in legs:
        walk(travel_time_s(p, distance_m), distance_m, payload_kg, p.drive_load)
    if dwell_s > 0.0:
        walk(dwell_s, 0.0, 0.0, p.dwell_load)
    return sim, peak
