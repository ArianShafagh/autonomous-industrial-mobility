"""Gymnasium wrapper around FactorySim, for the reinforcement-learning comparison (WP6).

One `step()` = one whole action (drive + load / unload / charge / wait), which is why an episode
is a few dozen steps rather than thousands: the decision problem is "what next", not "which wheel
speed". Nav2 and the physics remain the robot's job.

Action space (fixed, same order everywhere):
    PICKUP A | PICKUP B | PICKUP C | DELIVER | CHARGE to each configured target | WAIT

`action_masks()` returns which of them are currently legal (sb3-contrib MaskablePPO uses it, and
the symbolic layer of the neuro-symbolic model uses the same mask), so no model ever has to learn
that delivering an empty load is pointless.

Observation (float32, all scaled to roughly 0..1): robot battery/temperature/condition/payload,
location one-hot, shift time left, and per section: buffer fill, actual rate, time to full,
health, status one-hot, distance, units that fit.
"""
import gymnasium as gym
import numpy as np
from gymnasium import spaces

from robofetch_core.mission_plan import CHARGE, DELIVER, PICKUP, WAIT, Action

from robofetch_ai.env.factory_sim import LOCATIONS, FactorySim

STATUSES = ("RUNNING", "DEGRADED", "BLOCKED", "FAULT")
SECTION_FEATURES = 6 + len(STATUSES)          # fill, rate, time_to_full, health, distance, fits
ROBOT_FEATURES = 5 + len(LOCATIONS)           # battery, temp, condition, payload, time left


class FactoryEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, scenario="balanced", seed=None, config_dir=None, overrides=None,
                 shift_duration_s=None, randomise_seed=True):
        super().__init__()
        # A list of scenarios = a different, randomly drawn scenario at every reset, so a single
        # environment covers all of them (a fixed scenario per parallel env left PPO seeing only
        # as many scenarios as it had envs - found in WP9).
        self.scenarios = [scenario] if isinstance(scenario, str) else list(scenario)
        self.sim_kwargs = dict(config_dir=config_dir, overrides=overrides,
                               shift_duration_s=shift_duration_s)
        self.randomise_seed = randomise_seed
        self._rng = np.random.default_rng(seed)
        self.sim = FactorySim(scenario=self.scenarios[0], seed=seed, **self.sim_kwargs)
        self.sections = list(self.sim.sections)
        self.actions = ([Action(PICKUP, sid) for sid in self.sections]
                        + [Action(DELIVER)]
                        + [Action(CHARGE, value=t) for t in self.sim.charge_targets()]
                        + [Action(WAIT, value=float(self.sim.sim_cfg["wait_slice_s"]))])
        self.action_space = spaces.Discrete(len(self.actions))
        size = ROBOT_FEATURES + len(self.sections) * SECTION_FEATURES
        self.observation_space = spaces.Box(low=-1.0, high=2.0, shape=(size,), dtype=np.float32)

    # ---------------------------------------------------------------------------- gym API
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is None and self.randomise_seed:
            seed = int(self._rng.integers(0, 2**31 - 1))
        scenario = (self.scenarios[0] if len(self.scenarios) == 1
                    else self.scenarios[int(self._rng.integers(len(self.scenarios)))])
        self.sim = FactorySim(scenario=scenario, seed=seed, **self.sim_kwargs)
        return self._observation(), {}

    def step(self, action_index):
        action = self.actions[int(action_index)]
        legal = {str(a) for a in self.sim.legal_actions()}
        outcome = self.sim.step(action)
        reward = self.sim.reward(outcome)
        if str(action) not in legal:
            # Illegal choices are executed anyway (they simply achieve nothing) but cost a little,
            # so an unmasked agent still learns the rules instead of exploiting them.
            reward -= 0.1
        terminated = self.sim.done
        info = {"outcome": outcome, "legal": str(action) in legal}
        if terminated:
            info["summary"] = self.sim.summary()
        return self._observation(), float(reward), terminated, False, info

    def action_masks(self):
        legal = {str(a) for a in self.sim.legal_actions()}
        return np.array([str(a) in legal for a in self.actions], dtype=bool)

    # ------------------------------------------------------------------------- observation
    def _observation(self):
        s = self.sim.state()
        p = self.sim.p
        longest = max(max(row.values()) for row in self.sim.matrix.values()) or 1.0
        horizon = 600.0                                     # scale for "time to full"
        obs = [s["battery_percent"] / 100.0,
               (s["temperature_c"] - p.ambient_c) / max(1.0, p.max_c - p.ambient_c),
               s["condition_percent"] / 100.0,
               s["payload_kg"] / p.max_payload_kg,
               s["time_left_s"] / max(1.0, self.sim.shift_duration_s)]
        obs += [1.0 if s["location"] == name else 0.0 for name in LOCATIONS]
        for sid in self.sections:
            sec = s["sections"][sid]
            obs += [sec["buffer_fill"],
                    sec["rate_actual_per_hour"] / max(1.0, sec["rate_nominal_per_hour"]),
                    min(1.0, sec["time_to_full_s"] / horizon) if np.isfinite(sec["time_to_full_s"])
                    else 1.0,
                    sec["health_percent"] / 100.0,
                    sec["distance_m"] / longest,
                    sec["units_that_fit"] / max(1, sec["buffer_capacity"])]
            obs += [1.0 if sec["status"] == st else 0.0 for st in STATUSES]
        return np.asarray(obs, dtype=np.float32)
