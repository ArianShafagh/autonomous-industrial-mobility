"""System tests for the production section model.

They pin the behaviour every decision model relies on: nominal rates are really produced, a
full buffer stops the line and counts lost production, faults stop it and repair restores it,
the robot can never take more than it may carry, and a seed reproduces a run exactly.
"""
import math
import os

import pytest

from robofetch_factory.factory_model import (BLOCKED, DEGRADED, FAULT, RUNNING, Section,
                                             SectionParams, build_sections, load_config)

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")


def quiet(rate=20.0, capacity=10_000, **kw):
    """A section with no noise, no wear and no faults unless a test asks for them."""
    base = dict(rate_per_hour=rate, buffer_capacity=capacity, unit_mass_kg=0.5, initial_buffer=0,
                production_cv=0.0, health_initial=100.0, wear_per_unit=0.0, degraded_below=60.0,
                min_rate_factor=0.5, faults_per_hour=0.0, fault_wear_gain=3.0,
                repair_minutes=20.0, repair_cv=0.3)
    base.update(kw)
    return SectionParams(**base)


def run(section, sim_seconds, dt=1.0):
    for _ in range(int(round(sim_seconds / dt))):
        section.step(dt)


# ------------------------------------------------------------------------ production rate
@pytest.mark.parametrize("sid,rate", [("A", 20.0), ("B", 30.0), ("C", 5.0)])
def test_configured_rates_are_the_user_requirement(sid, rate):
    cfg = load_config("balanced", CONFIG_DIR)
    assert cfg["factory"]["sections"][sid]["rate_per_hour"] == rate


@pytest.mark.parametrize("rate", [5.0, 20.0, 30.0])
def test_noise_free_section_produces_exactly_its_rate(rate):
    s = Section("X", quiet(rate), time_scale=1.0)
    run(s, 10 * 3600, dt=5.0)                      # 10 factory hours
    assert s.produced_total == pytest.approx(10 * rate, abs=1)


def test_time_scale_compresses_factory_time():
    s = Section("X", quiet(30.0), time_scale=20.0)
    run(s, 180.0)                                   # 3 sim minutes = 1 factory hour
    assert s.produced_total == pytest.approx(30, abs=1)


def test_noisy_rate_is_right_on_average():
    s = Section("X", quiet(30.0, production_cv=0.3), time_scale=1.0, seed=7)
    run(s, 100 * 3600, dt=10.0)
    assert s.produced_total == pytest.approx(3000, rel=0.05)


def test_result_does_not_depend_on_step_size():
    coarse = Section("X", quiet(20.0, production_cv=0.2), time_scale=20.0, seed=3)
    fine = Section("X", quiet(20.0, production_cv=0.2), time_scale=20.0, seed=3)
    run(coarse, 600.0, dt=2.0)
    run(fine, 600.0, dt=0.1)
    assert coarse.produced_total == fine.produced_total


# ------------------------------------------------------------------------ buffer / blocking
def test_full_buffer_blocks_and_counts_lost_production():
    s = Section("X", quiet(36.0, capacity=5), time_scale=1.0)   # one unit per 100 s
    run(s, 500.0)
    assert s.buffer == 5 and s.status == BLOCKED
    produced = s.produced_total
    run(s, 1000.0)                                  # 10 more units' worth while blocked
    assert s.produced_total == produced
    assert s.lost_units == pytest.approx(10.0, abs=0.1)
    assert s.blocked_time_s == pytest.approx(1000.0, abs=1.0)
    assert s.time_to_full_sim_s() == 0.0


def test_pickup_unblocks_the_line():
    s = Section("X", quiet(36.0, capacity=5), time_scale=1.0)
    run(s, 600.0)
    assert s.status == BLOCKED
    assert s.pickup() == 5
    run(s, 150.0)
    assert s.status == RUNNING and s.buffer >= 1


def test_time_to_full_matches_what_actually_happens():
    s = Section("X", quiet(36.0, capacity=8, initial_buffer=2), time_scale=2.0)
    run(s, 30.0)
    predicted = s.time_to_full_sim_s()
    elapsed = 0.0
    while s.status != BLOCKED:
        s.step(0.5)
        elapsed += 0.5
    assert elapsed == pytest.approx(predicted, abs=0.5)


# ------------------------------------------------------------------------------- pickup
def test_pickup_respects_robot_payload_limit():
    s = Section("X", quiet(capacity=20, initial_buffer=15, unit_mass_kg=0.4))
    assert s.pickup(max_mass_kg=5.0) == 12          # 12 * 0.4 = 4.8 kg
    assert s.buffer == 3 and s.picked_total == 12


def test_pickup_never_takes_more_than_available_or_negative():
    s = Section("X", quiet(capacity=20, initial_buffer=2))
    assert s.pickup(max_units=10) == 2
    assert s.pickup(max_units=-3) == 0
    assert s.buffer == 0


# ------------------------------------------------------------------------ health / faults
def test_wear_degrades_and_slows_the_line():
    s = Section("X", quiet(3600.0, wear_per_unit=1.0, degraded_below=60.0,
                           min_rate_factor=0.5), time_scale=1.0)
    run(s, 45.0)                                    # 45 units -> health 55
    assert s.status == DEGRADED
    assert s.rate_factor() == pytest.approx(0.5 + 0.5 * 0.55, abs=0.01)
    assert s.actual_rate_per_hour() < 3600.0


def test_fault_stops_production_and_repair_restores_health():
    s = Section("X", quiet(36.0, faults_per_hour=3600.0, repair_minutes=10.0, repair_cv=0.0,
                           health_initial=70.0), time_scale=1.0, seed=1)
    s.step(1.0)                                     # hazard ~1/s -> fault almost surely
    assert s.status == FAULT and s.faults_total == 1
    assert s.time_to_full_sim_s() == math.inf
    produced = s.produced_total
    s.params.faults_per_hour = 0.0
    run(s, 590.0)
    assert s.status == FAULT and s.produced_total == produced
    run(s, 20.0)
    assert s.status == RUNNING and s.health == 100.0
    assert s.fault_time_s == pytest.approx(600.0, abs=1.0)


def test_worn_machines_fail_sooner():
    """Hazard at health 40 is (1 + 3 * 0.6) = 2.8x nominal, so the first fault comes ~2.8x
    sooner. (Measured to the FIRST fault: a repair services the machine back to health 100.)"""
    def mean_time_to_first_fault(health):
        times = []
        for seed in range(200):
            s = Section("X", quiet(10.0, faults_per_hour=0.5, fault_wear_gain=3.0,
                                   health_initial=health), time_scale=1.0, seed=seed)
            t = 0.0
            while s.faults_total == 0:
                s.step(60.0)
                t += 60.0
            times.append(t)
        return sum(times) / len(times)
    ratio = mean_time_to_first_fault(100.0) / mean_time_to_first_fault(40.0)
    assert ratio == pytest.approx(2.8, rel=0.25)


# --------------------------------------------------------------------- config / scenarios
def test_same_seed_reproduces_the_shift_and_other_seed_differs():
    cfg = load_config("fault_burst", CONFIG_DIR)

    def shift(seed):
        secs = build_sections(cfg, seed)
        for _ in range(1800):
            for s in secs.values():
                s.step(1.0)
        return {k: (s.produced_total, s.faults_total, round(s.lost_units, 6))
                for k, s in secs.items()}

    assert shift(5) == shift(5)
    assert shift(5) != shift(6)


def test_sections_get_independent_noise_streams():
    a = load_config("balanced", CONFIG_DIR)["factory"]["sections"]["A"]
    cfg = load_config("balanced", CONFIG_DIR, overrides={"factory": {"sections": {"B": dict(a)}}})
    secs = build_sections(cfg, 1)
    trace = {"A": [], "B": []}
    for _ in range(600):
        for sid in trace:
            secs[sid].step(1.0)
            trace[sid].append(secs[sid].produced_total)
    assert trace["A"] != trace["B"]


SCENARIOS = sorted(f[:-5] for f in os.listdir(os.path.join(CONFIG_DIR, "scenarios")))


@pytest.mark.parametrize("name", SCENARIOS)
def test_every_scenario_loads_and_runs_a_shift(name):
    cfg = load_config(name, CONFIG_DIR)
    secs = build_sections(cfg)
    assert set(secs) == {"A", "B", "C"}
    for _ in range(int(cfg["time"]["shift_duration_s"])):
        for s in secs.values():
            s.step(1.0)
    for s in secs.values():
        snap = s.snapshot()
        assert 0.0 <= snap["buffer_fill"] <= 1.0
        assert snap["produced_total"] >= 0


def test_scenario_overrides_only_what_it_names():
    base = load_config("balanced", CONFIG_DIR)
    hot = load_config("one_hot_section", CONFIG_DIR)
    hs, bs = hot["factory"]["sections"], base["factory"]["sections"]
    assert hs["B"]["rate_per_hour"] == 60.0
    assert hs["B"]["unit_mass_kg"] == bs["B"]["unit_mass_kg"]
    assert hs["A"] == bs["A"]
    assert hot["robot"] == base["robot"]


def test_unknown_scenario_is_rejected():
    with pytest.raises(ValueError, match="unknown scenario"):
        load_config("no_such_scenario", CONFIG_DIR)


def test_misspelled_parameter_is_rejected():
    with pytest.raises(ValueError, match="unknown parameter 'params.robot.battery.capacity_w'"):
        load_config("balanced", CONFIG_DIR,
                    overrides={"robot": {"battery": {"capacity_w": 30.0}}})


@pytest.mark.parametrize("name", SCENARIOS)
def test_a_full_buffer_always_fits_on_the_robot(name):
    """Each section's full buffer must be collectable in one trip, or the robot can never
    unblock that line in a single visit."""
    cfg = load_config(name, CONFIG_DIR)
    limit = cfg["robot"]["handling"]["max_payload_kg"]
    for sid, sec in cfg["factory"]["sections"].items():
        assert sec["buffer_capacity"] * sec["unit_mass_kg"] <= limit, sid
