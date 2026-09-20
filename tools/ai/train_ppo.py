#!/usr/bin/env python3
"""Train the PPO agent that the neuro-symbolic model is compared against (WP6).

Same environment, same actions, same objective as the neuro-symbolic model. MaskablePPO is used
so the agent is not forced to waste training on obviously impossible actions - the same mask the
symbolic layer applies. `--no-masks` trains without it, which shows how much of the symbolic
layer's benefit is simply "knowing the rules".

    venv/bin/python -u tools/ai/train_ppo.py                     # settings from params.yaml
    venv/bin/python -u tools/ai/train_ppo.py --timesteps 50000 --threads 2
"""
import argparse
import functools
import json
import os
import sys
import time

print = functools.partial(print, flush=True)  # noqa: A001

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for pkg in ("robofetch_core", "robofetch_factory", "robofetch_ai"):
    sys.path.insert(0, os.path.join(WS, "src", pkg))

CONFIG_DIR = os.path.join(WS, "src", "robofetch_factory", "config")
RESULTS_DIR = os.path.join(WS, "tools", "ai", "results")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=None)
    ap.add_argument("--threads", type=int, default=2, help="torch threads (keep low when other "
                                                           "training runs at the same time)")
    ap.add_argument("--no-masks", action="store_true", help="train without the action mask")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    import torch
    torch.set_num_threads(args.threads)
    from sb3_contrib import MaskablePPO
    from sb3_contrib.common.maskable.utils import get_action_masks  # noqa: F401
    from sb3_contrib.common.wrappers import ActionMasker
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv

    from robofetch_factory.factory_model import load_config
    from robofetch_ai.env.factory_env import FactoryEnv
    from robofetch_ai.policies.rl_ppo import DEFAULT_MODEL

    cfg = load_config("balanced", CONFIG_DIR)
    rl = dict(cfg["mission"]["rl"])
    if args.timesteps:
        rl["total_timesteps"] = args.timesteps
    if args.no_masks:
        rl["use_action_masks"] = False
    scenarios = list(cfg["mission"]["evaluation"]["training_scenarios"])

    def make(rank):
        def _make():
            # Every parallel environment draws a random training scenario at each reset, so all
            # six are seen however many envs there are. Seeded per rank for reproducibility.
            env = FactoryEnv(scenario=scenarios, seed=int(rl["seed"]) + rank, config_dir=CONFIG_DIR)
            if rl["use_action_masks"]:
                env = ActionMasker(env, lambda e: e.unwrapped.action_masks())
            return Monitor(env)
        return _make

    envs = DummyVecEnv([make(i) for i in range(int(rl["n_envs"]))])
    model = MaskablePPO("MlpPolicy", envs, verbose=1, device="cpu", seed=int(rl["seed"]),
                        n_steps=int(rl["n_steps"]), batch_size=int(rl["batch_size"]),
                        learning_rate=float(rl["learning_rate"]), gamma=float(rl["gamma"]),
                        gae_lambda=float(rl["gae_lambda"]), ent_coef=float(rl["ent_coef"]),
                        policy_kwargs={"net_arch": list(rl["net_arch"])})

    print(f"training PPO for {rl['total_timesteps']} steps on {scenarios} "
          f"({'masked' if rl['use_action_masks'] else 'UNMASKED'}, {args.threads} threads)")
    started = time.time()
    model.learn(total_timesteps=int(rl["total_timesteps"]), progress_bar=False)
    training_s = time.time() - started

    out = args.out or (DEFAULT_MODEL if rl["use_action_masks"]
                       else DEFAULT_MODEL.replace(".zip", "_unmasked.zip"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    model.save(out)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    report = os.path.join(RESULTS_DIR, time.strftime("train_ppo_%Y%m%d_%H%M%S.json"))
    with open(report, "w") as fh:
        json.dump({"config": rl, "scenarios": scenarios, "training_seconds": round(training_s, 1),
                   "timesteps": int(rl["total_timesteps"])}, fh, indent=2)
    print(f"\ntrained in {training_s / 60:.1f} min")
    print(f"model  -> {os.path.relpath(out, WS)}")
    print(f"report -> {os.path.relpath(report, WS)}")


if __name__ == "__main__":
    main()
