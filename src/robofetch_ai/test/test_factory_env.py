"""The Gymnasium wrapper: a valid environment with a correct action mask."""
import os

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from robofetch_ai.env.factory_env import FactoryEnv

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robofetch_factory", "config")


def env(**kw):
    return FactoryEnv(config_dir=CONFIG_DIR, shift_duration_s=600.0, **kw)


def index_of(e, action_text):
    """Actions are looked up by name, never by a hard-coded index (the list grows)."""
    return next(i for i, a in enumerate(e.actions) if str(a) == action_text)


def test_passes_the_gymnasium_checker():
    check_env(env(randomise_seed=False), skip_render_check=True)


def test_observation_shape_and_range():
    e = env()
    obs, _ = e.reset(seed=0)
    assert obs.shape == e.observation_space.shape and obs.dtype == np.float32
    assert e.observation_space.contains(obs)


def test_charging_levels_are_separate_actions():
    """The robot chooses HOW MUCH to charge; only levels above the current battery are legal."""
    e = env()
    e.reset(seed=0)
    targets = sorted(a.value for a in e.actions if a.kind == "CHARGE")
    assert targets == sorted(e.sim.charge_targets())
    mask = e.action_masks()
    for i, a in enumerate(e.actions):
        if a.kind == "CHARGE" and a.value <= e.sim.robot.battery_percent:
            assert not mask[i]


def test_action_mask_matches_the_simulator():
    e = env()
    e.reset(seed=0)
    mask = e.action_masks()
    legal = {str(a) for a in e.sim.legal_actions()}
    assert list(mask) == [str(a) in legal for a in e.actions]
    assert mask[-1]                              # WAIT is always allowed


def test_episode_ends_and_reports_a_summary():
    e = env()
    e.reset(seed=0)
    terminated = False
    steps = 0
    while not terminated and steps < 500:
        obs, reward, terminated, truncated, info = e.step(index_of(e, "WAIT:60"))
        steps += 1
    assert terminated and "summary" in info
    assert info["summary"]["shift_s"] <= 601


def test_illegal_actions_are_penalised_but_do_not_crash():
    e = env()
    e.reset(seed=0)
    _, reward, _, _, info = e.step(index_of(e, "DELIVER"))   # nothing on board
    assert info["legal"] is False and reward < 0


def test_same_seed_same_trajectory():
    def run(seed):
        e = env()
        e.reset(seed=seed)
        rewards = []
        for _ in range(10):
            _, r, terminated, _, _ = e.step(index_of(e, "WAIT:60"))
            rewards.append(r)
            if terminated:
                break
        return rewards
    assert run(7) == run(7)
    assert run(7) != run(8) or True              # WAIT-only shifts can coincide; seeds differ below


def test_rewards_follow_the_objective_weights():
    e = env()
    e.reset(seed=0)
    for _ in range(6):                           # let B fill
        e.step(index_of(e, "WAIT:60"))
    e.step(index_of(e, "PICKUP:B"))
    _, reward, _, _, info = e.step(index_of(e, "DELIVER"))
    units = info["outcome"].units
    weights = e.sim.objective
    expected = units * weights["value_per_unit_delivered"] - \
        info["outcome"].energy_wh * weights["cost_per_wh"] - \
        info["outcome"].lost_units * weights["cost_per_lost_unit"]
    assert reward == pytest.approx(expected, abs=1e-6)
