"""Production section model — pure Python, no ROS.

Shared by the live section nodes (Gazebo runs) and the fast training/evaluation simulator, so
both run the SAME production dynamics. Deterministic for a given seed and step sequence.

A section has a machine that turns "work" into finished units, and an output buffer the robot
empties. Four things interact:

    production  a unit needs Gamma-distributed work (mean 1, cv = production_cv); the machine
                accumulates work at rate_per_hour * health factor (factory time)
    buffer      finished units wait here; when it is FULL the machine stops (BLOCKED) and the
                production it would have made is counted as lost
    health      every produced unit wears the machine; below `degraded_below` it runs slower
    faults      random, more likely when worn; the machine stops for a random repair time and
                comes back serviced (health 100)

Time: all `step()` and query arguments are SIM seconds. `time_scale` converts them to factory
time internally, so callers never mix the two clocks.
"""
import copy
import math
import os
import random
from dataclasses import dataclass, field

import yaml

RUNNING, BLOCKED, DEGRADED, FAULT = "RUNNING", "BLOCKED", "DEGRADED", "FAULT"
STATUS_CODES = {RUNNING: 0, DEGRADED: 1, BLOCKED: 2, FAULT: 3}


@dataclass
class SectionParams:
    """One section's parameters. No defaults on purpose: every value comes from params.yaml."""
    rate_per_hour: float
    buffer_capacity: int
    unit_mass_kg: float
    initial_buffer: int
    production_cv: float
    health_initial: float
    wear_per_unit: float
    degraded_below: float
    min_rate_factor: float
    faults_per_hour: float
    fault_wear_gain: float
    repair_minutes: float
    repair_cv: float


def _gamma(rng, mean, cv):
    """Gamma sample with the given mean and coefficient of variation (cv 0 -> exactly mean)."""
    if cv <= 0.0:
        return mean
    shape = 1.0 / (cv * cv)
    return rng.gammavariate(shape, mean / shape)


@dataclass
class Section:
    section_id: str
    params: SectionParams
    time_scale: float = 1.0
    seed: int = 0

    # --- state (all counters in units, times in FACTORY seconds unless named _sim)
    buffer: int = field(init=False)
    health: float = field(init=False)
    produced_total: int = field(init=False, default=0)
    picked_total: int = field(init=False, default=0)
    lost_units: float = field(init=False, default=0.0)
    blocked_time_s: float = field(init=False, default=0.0)
    fault_time_s: float = field(init=False, default=0.0)
    faults_total: int = field(init=False, default=0)
    factory_time_s: float = field(init=False, default=0.0)
    repair_remaining_s: float = field(init=False, default=0.0)

    def __post_init__(self):
        p = self.params
        # Independent stream per section: the same seed must not give A and B identical noise.
        self.rng = random.Random(f"{self.seed}:{self.section_id}")
        self.buffer = min(int(p.initial_buffer), int(p.buffer_capacity))
        self.health = float(p.health_initial)
        self._progress = 0.0
        self._work_needed = _gamma(self.rng, 1.0, p.production_cv)

    # ------------------------------------------------------------------------------ status
    @property
    def status(self):
        if self.repair_remaining_s > 0.0:
            return FAULT
        if self.buffer >= self.params.buffer_capacity:
            return BLOCKED
        if self.health < self.params.degraded_below:
            return DEGRADED
        return RUNNING

    def rate_factor(self):
        """Fraction of nominal speed the machine currently runs at (health effect only)."""
        p = self.params
        if self.health >= p.degraded_below:
            return 1.0
        return p.min_rate_factor + (1.0 - p.min_rate_factor) * self.health / 100.0

    def actual_rate_per_hour(self):
        """What the machine is producing right now (0 when stopped)."""
        if self.status in (FAULT, BLOCKED):
            return 0.0
        return self.params.rate_per_hour * self.rate_factor()

    def time_to_full_sim_s(self):
        """Expected SIM seconds until the buffer is full at the current rate (inf if stopped
        by a fault; 0 if already full). Counts the partly finished unit on the machine."""
        free = self.params.buffer_capacity - self.buffer
        if free <= 0:
            return 0.0
        rate = self.params.rate_per_hour * self.rate_factor()
        if self.status == FAULT or rate <= 0.0:
            return math.inf
        units_of_work = max(0.0, free - 1 + (self._work_needed - self._progress))
        return units_of_work / (rate / 3600.0) / self.time_scale

    def fault_remaining_sim_s(self):
        return self.repair_remaining_s / self.time_scale

    # ------------------------------------------------------------------------------ dynamics
    def step(self, dt_sim):
        """Advance by `dt_sim` simulation seconds."""
        if dt_sim <= 0.0:
            return
        dt = dt_sim * self.time_scale
        p = self.params
        self.factory_time_s += dt

        if self.repair_remaining_s > 0.0:
            used = min(dt, self.repair_remaining_s)
            self.repair_remaining_s -= used
            self.fault_time_s += used
            dt -= used
            if self.repair_remaining_s > 0.0:
                return
            self.health = 100.0          # repaired and serviced
            if dt <= 0.0:
                return

        # Fault arrival over the remaining interval (hazard grows with wear).
        hazard_per_s = p.faults_per_hour / 3600.0 * (
            1.0 + p.fault_wear_gain * (1.0 - self.health / 100.0))
        if hazard_per_s > 0.0 and self.rng.random() < 1.0 - math.exp(-hazard_per_s * dt):
            self.faults_total += 1
            self.repair_remaining_s = _gamma(self.rng, p.repair_minutes * 60.0, p.repair_cv)
            return

        rate_per_s = p.rate_per_hour / 3600.0
        # Produce unit by unit so wear, the rate factor and blocking update exactly when a unit
        # completes - this keeps results independent of the step size.
        while dt > 0.0:
            if self.buffer >= p.buffer_capacity:
                self.blocked_time_s += dt
                self.lost_units += rate_per_s * self.rate_factor() * dt
                return
            speed = rate_per_s * self.rate_factor()
            if speed <= 0.0:
                return
            remaining_work = self._work_needed - self._progress
            t_unit = remaining_work / speed
            if t_unit > dt:
                self._progress += speed * dt
                return
            dt -= t_unit
            self.buffer += 1
            self.produced_total += 1
            self.health = max(0.0, self.health - p.wear_per_unit)
            self._progress = 0.0
            self._work_needed = _gamma(self.rng, 1.0, p.production_cv)

    def pickup(self, max_units=None, max_mass_kg=None):
        """Remove finished units for the robot. Returns the number of units taken."""
        take = self.buffer
        if max_units is not None:
            take = min(take, max(0, int(max_units)))
        if max_mass_kg is not None and self.params.unit_mass_kg > 0.0:
            take = min(take, max(0, int(math.floor(max_mass_kg / self.params.unit_mass_kg + 1e-9))))
        self.buffer -= take
        self.picked_total += take
        return take

    def snapshot(self):
        """Everything a monitor or a decision model may look at, as plain data."""
        ttf = self.time_to_full_sim_s()
        return {
            "section_id": self.section_id,
            "status": self.status,
            "status_code": STATUS_CODES[self.status],
            "factory_time_s": round(self.factory_time_s, 3),
            "produced_total": self.produced_total,
            "picked_total": self.picked_total,
            "buffer_units": self.buffer,
            "buffer_capacity": self.params.buffer_capacity,
            "buffer_fill": self.buffer / self.params.buffer_capacity,
            "unit_mass_kg": self.params.unit_mass_kg,
            "buffer_mass_kg": self.buffer * self.params.unit_mass_kg,
            "rate_nominal_per_hour": self.params.rate_per_hour,
            "rate_actual_per_hour": self.actual_rate_per_hour(),
            "health_percent": self.health,
            "time_to_full_s": ttf,
            "fault_remaining_s": self.fault_remaining_sim_s(),
            "faults_total": self.faults_total,
            "lost_units": self.lost_units,
            "blocked_time_s": self.blocked_time_s / self.time_scale,
        }


# ---------------------------------------------------------------------------------- config
def deep_merge(base, override):
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def check_keys(base, override, where="params"):
    """Every key a scenario sets must exist in params.yaml, so a typo fails loudly instead of
    silently leaving the default in place."""
    for key, value in (override or {}).items():
        if key not in base:
            raise ValueError(f"unknown parameter '{where}.{key}' (not in params.yaml)")
        if isinstance(value, dict) and isinstance(base[key], dict):
            check_keys(base[key], value, f"{where}.{key}")


def load_config(scenario="balanced", config_dir=None, overrides=None):
    """params.yaml merged with scenarios/<scenario>.yaml and optional dict overrides.

    Returns the full nested dict: cfg["time"], cfg["robot"], cfg["factory"], plus "name" and
    "description" of the scenario.
    """
    if config_dir is None:
        from robofetch_factory.layout import config_dir as _config_dir
        config_dir = _config_dir()
    with open(os.path.join(config_dir, "params.yaml")) as fh:
        base = yaml.safe_load(fh)
    path = os.path.join(config_dir, "scenarios", f"{scenario}.yaml")
    if not os.path.isfile(path):
        known = sorted(f[:-5] for f in os.listdir(os.path.join(config_dir, "scenarios")))
        raise ValueError(f"unknown scenario '{scenario}', known: {', '.join(known)}")
    with open(path) as fh:
        scen = yaml.safe_load(fh) or {}
    meta = {k: scen.pop(k) for k in ("name", "description") if k in scen}
    check_keys(base, scen)
    check_keys(base, overrides)
    cfg = deep_merge(deep_merge(base, scen), overrides)
    cfg["name"] = meta.get("name", scenario)
    cfg["description"] = meta.get("description", "")
    return cfg


def section_params(cfg, section_id):
    factory = cfg["factory"]
    merged = deep_merge(factory.get("section_defaults", {}), factory["sections"][section_id])
    return SectionParams(**merged)


def build_sections(cfg, seed=None):
    seed = cfg["time"].get("seed", 0) if seed is None else seed
    return {sid: Section(sid, section_params(cfg, sid), float(cfg["time"]["time_scale"]), seed)
            for sid in cfg["factory"]["sections"]}
