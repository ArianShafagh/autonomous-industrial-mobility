"""The reinforcement-learning comparison: a PPO agent behind the same Policy interface.

It sees the same environment, the same action set and the same objective as the neuro-symbolic
model, so any difference in the results is about the METHOD, not the problem.

Two honest differences, which are the point of the comparison:
  * it has no symbolic layer, so nothing guarantees it keeps the battery reserve or stays under
    the temperature limit - it can only learn to;
  * it cannot explain a decision. `explanation` reports the action probability, which is the most
    a policy network can say about why.

Trained by tools/ai/train_ppo.py; without a trained model this policy refuses to run (rather than
silently behaving randomly).
"""
import os
import time

import numpy as np

from robofetch_ai.policies.base import Decision, Policy

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "models", "ppo_policy.zip")


class PPOPolicy(Policy):
    name = "ppo"

    def __init__(self, cfg, model_path=DEFAULT_MODEL, env=None, model=None, use_masks=None):
        from sb3_contrib import MaskablePPO                      # imported lazily: heavy

        self.cfg = cfg
        if model is None:
            if not os.path.exists(model_path):
                raise FileNotFoundError(
                    f"no trained PPO model at {model_path} - run tools/ai/train_ppo.py")
            model = MaskablePPO.load(model_path, device="cpu")
        self.model = model
        self.use_masks = cfg["mission"]["rl"]["use_action_masks"] if use_masks is None else use_masks
        # The action list and the observation layout come from the environment definition, so the
        # live robot and the training environment cannot drift apart.
        from robofetch_ai.env.factory_env import FactoryEnv
        self.env = env or FactoryEnv(config_dir=None)
        self.actions = self.env.actions

    def decide(self, state, legal_actions, sim=None):
        started = time.perf_counter()
        self.env.sim = sim                        # observe the caller's simulator
        obs = self.env._observation()
        masks = self.env.action_masks() if self.use_masks else None
        index, _ = self.model.predict(obs, action_masks=masks, deterministic=True)
        action = self.actions[int(index)]
        legal = {str(a) for a in legal_actions}
        if str(action) not in legal:
            # An unmasked agent can ask for something impossible; the robot must still do
            # something sensible, and the fallback is reported honestly.
            fallback = next((a for a in legal_actions if a.kind == "WAIT"), legal_actions[0])
            return Decision(fallback, f"PPO chose {action}, which is not possible now; waiting",
                            {}, (time.perf_counter() - started) * 1000.0)
        probabilities = self._probabilities(obs, masks)
        return Decision(action,
                        f"PPO policy (probability {probabilities.get(str(action), float('nan')):.2f}"
                        f"{'' if self.use_masks else ', unmasked'})",
                        probabilities, (time.perf_counter() - started) * 1000.0)

    def _probabilities(self, obs, masks):
        import torch
        with torch.no_grad():
            tensor, _ = self.model.policy.obs_to_tensor(obs)
            distribution = self.model.policy.get_distribution(
                tensor, action_masks=None if masks is None else np.asarray(masks))
            probs = distribution.distribution.probs.squeeze(0).tolist()
        return {str(a): round(float(p), 3) for a, p in zip(self.actions, probs)}
