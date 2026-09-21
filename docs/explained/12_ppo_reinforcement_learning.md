# 12 — PPO: the deep reinforcement-learning comparison

Files:
- `src/robofetch_ai/robofetch_ai/policies/rl_ppo.py` (73 lines): `PPOPolicy`
- `tools/ai/train_ppo.py` (97 lines)
- `src/robofetch_ai/robofetch_ai/env/factory_env.py`: the environment (file 08 §6)
- `src/robofetch_ai/robofetch_ai/models/ppo_policy.zip`: the trained model
- `params.yaml → mission.rl`
- Libraries: **Stable-Baselines3 2.9** and **sb3-contrib** (`MaskablePPO`), **Gymnasium 1.3**, **PyTorch 2.14 (CPU)**

This file explains reinforcement learning from the ground up, then exactly how this project uses PPO, and then the problems that affect the fairness of the comparison.

---

## 1. Reinforcement learning in five ideas

### 1.1 Agent, environment, reward

At each step the agent observes $o_t$, takes action $a_t$, and receives reward $r_t$ and the next observation. It is never told the right action. It must discover which behaviour leads to high **return**:

$$G_t = \sum_{k=0}^{\infty} \gamma^k r_{t+k}, \qquad 0 \le \gamma < 1$$

Here one step is a whole macro-action (about 62 s on average under the symbolic policy in balanced), and one episode is a shift of about 45–65 steps.

### 1.2 Policy and value functions

- **Policy** $\pi_\theta(a \mid o)$: a neural network outputting a probability for each of the 8 actions.
- **State value** $V^\pi(s) = \mathbb{E}_\pi[G_t \mid s_t = s]$: how good a situation is under the policy.
- **Action value** $Q^\pi(s, a)$: how good it is to take $a$ and then follow $\pi$.
- **Advantage** $A^\pi(s, a) = Q^\pi(s, a) - V^\pi(s)$: how much better $a$ is than the policy's average.

### 1.3 Policy gradient

To improve $\pi_\theta$ directly, push up the log-probability of actions that turned out better than average:

$$\nabla_\theta J(\theta) = \mathbb{E}\left[\nabla_\theta \log \pi_\theta(a_t \mid s_t)\, \hat{A}_t\right]$$

This is the **policy gradient theorem** (REINFORCE uses the return $G_t$ in place of $\hat{A}_t$, which is unbiased but very noisy).

### 1.4 Actor-critic

Use a second network, the **critic** $V_\phi(s)$, to estimate values. The advantage estimate then has much less variance than raw returns. The **actor** is the policy.

### 1.5 Generalised Advantage Estimation (GAE)

The one-step TD error is $\delta_t = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$. GAE mixes multi-step estimates with a parameter $\lambda$:

$$\hat{A}_t^{GAE(\gamma,\lambda)} = \sum_{l=0}^{\infty} (\gamma\lambda)^l\, \delta_{t+l}$$

$\lambda = 0$ gives the one-step TD advantage (low variance, biased by critic errors). $\lambda = 1$ gives the Monte-Carlo advantage (unbiased, high variance). This project uses **λ = 0.95**, the common default.

---

## 2. PPO: Proximal Policy Optimization

### 2.1 The problem PPO solves

Policy-gradient steps that are too large can destroy a good policy in one update, and data collected with the old policy then becomes misleading. **TRPO** constrains the KL divergence between old and new policy, but it is complex. **PPO** (Schulman et al., 2017) gets a similar effect with a simple clipped objective.

### 2.2 The clipped surrogate objective

With the probability ratio $\rho_t(\theta) = \dfrac{\pi_\theta(a_t \mid s_t)}{\pi_{\theta_{old}}(a_t \mid s_t)}$:

$$L^{CLIP}(\theta) = \mathbb{E}_t\left[\min\big(\rho_t \hat{A}_t,\ \text{clip}(\rho_t, 1-\epsilon, 1+\epsilon)\, \hat{A}_t\big)\right], \qquad \epsilon = 0.2$$

- If an action was **good** ($\hat{A} > 0$), increasing its probability helps only until $\rho = 1.2$. Beyond that there is no more gain, so there is no incentive to move further.
- If it was **bad** ($\hat{A} < 0$), decreasing its probability helps only until $\rho = 0.8$.

The full loss SB3 minimises:

$$\mathcal{L} = -L^{CLIP} + c_v \cdot \big(V_\phi(s_t) - \hat{G}_t\big)^2 - c_e \cdot \mathcal{H}\big[\pi_\theta(\cdot \mid s_t)\big]$$

- value coefficient $c_v$ = 0.5 (SB3 default)
- **entropy coefficient $c_e$ = `ent_coef` = 0.01**: rewards keeping some randomness, which prevents premature collapse to one action and keeps exploring

### 2.3 The training loop

```
repeat until total_timesteps:
    1. ROLLOUT: run the current policy in n_envs environments for n_steps each
       -> 4 × 512 = 2048 transitions (obs, action, reward, done, log-prob, value)
    2. compute GAE advantages and returns (γ = 0.995, λ = 0.95)
    3. UPDATE: n_epochs = 10 passes over the 2048 transitions in mini-batches of 256
       -> 8 mini-batches × 10 epochs = 80 gradient steps (Adam, lr 3e-4), clipped objective
    4. discard the data (on-policy), go to 1
```

300 000 timesteps / 2048 gives **about 146 update rounds**. At about 60 steps per shift that is **about 5 000 simulated shifts**. Training time was **475.7 s (7.9 min)** on CPU (`train_ppo_20260916_115450.json`).

**Concept: on-policy.** PPO's advantage estimates are valid only for data from the current policy, so old data is thrown away after each update. That makes PPO robust but sample-hungry, compared with off-policy methods (DQN, SAC) that reuse a replay buffer.

---

## 3. Action masking: MaskablePPO

**Concept: invalid action masking.** Before the softmax, logits of illegal actions are set to −∞, so they get probability 0:

$$\pi(a \mid s) = \frac{m_a\, e^{z_a}}{\sum_b m_b\, e^{z_b}}, \qquad m_a \in \{0, 1\}$$

The gradient then never pushes probability toward impossible actions, and exploration is spent only on meaningful choices. This is known to speed up learning substantially in discrete domains with many invalid actions (Huang & Ontañón, 2020).

In this project:

```python
env = ActionMasker(env, lambda e: e.unwrapped.action_masks())   # training
...
masks = self.env.action_masks() if self.use_masks else None       # inference
index, _ = self.model.predict(obs, action_masks=masks, deterministic=True)
```

The mask is the **pointless-action mask** (file 08 §3.6), not the safety rules. PPO therefore knows "don't deliver an empty load" but must **learn** "don't strand yourself".

---

## 4. The training script, line by line (`train_ppo.py`)

```python
torch.set_num_threads(args.threads)                 # default 2: small MLPs don't benefit from more
cfg = load_config("balanced", CONFIG_DIR); rl = cfg["mission"]["rl"]
scenarios = sorted(scenario files)

def make(rank):
    def _make():
        env = FactoryEnv(scenario=scenarios[rank % len(scenarios)], config_dir=CONFIG_DIR)
        if rl["use_action_masks"]:
            env = ActionMasker(env, lambda e: e.unwrapped.action_masks())
        return Monitor(env)                         # records episode reward/length for logging
    return _make

envs = DummyVecEnv([make(i) for i in range(rl["n_envs"])])        # 4 envs, run sequentially in-process
model = MaskablePPO("MlpPolicy", envs, device="cpu", seed=4242,
                    n_steps=512, batch_size=256, learning_rate=3e-4, gamma=0.995,
                    gae_lambda=0.95, ent_coef=0.01, policy_kwargs={"net_arch": [64, 64]})
model.learn(total_timesteps=300000)
model.save("models/ppo_policy.zip")                # or ppo_policy_unmasked.zip with --no-masks
```

### 4.1 The network that came out

Loaded from `ppo_policy.zip`:

```
MaskableActorCriticPolicy(
  policy_net: Linear(40, 64) → Tanh → Linear(64, 64) → Tanh     → action_net: Linear(64, 8)
  value_net : Linear(40, 64) → Tanh → Linear(64, 64) → Tanh     → value_net : Linear(64, 1)
)
total parameters: 14 153; n_epochs 10; clip_range 0.2
```

`net_arch: [64, 64]` gives **separate** actor and critic towers of that size (SB3 ≥ 1.8 semantics). SB3's default activation for PPO is **tanh**, while the neuro-symbolic scorer uses ReLU.

### 4.2 Hyperparameters and their meaning here

| Parameter | Value | Meaning in this problem |
|---|---|---|
| `total_timesteps` | 300 000 | about 5 000 shifts |
| `n_envs` | 4 | parallel shifts per rollout (see the bug in §6.1) |
| `n_steps` | 512 | ≈ 8–10 shifts per env per rollout |
| `batch_size` | 256 | mini-batch size |
| `learning_rate` | 3e-4 | Adam, constant |
| `gamma` | **0.995** | per decision. Effective horizon $1/(1-\gamma)$ = 200 steps, longer than a shift (~60 steps), so this is almost undiscounted within a shift |
| `gae_lambda` | 0.95 | advantage bias/variance trade-off |
| `ent_coef` | 0.01 | exploration bonus |
| `net_arch` | [64, 64] | "same size as the neuro-symbolic scorer" (the scorer is one 6 k network; PPO has two towers, 14 k in total) |
| `use_action_masks` | true | `--no-masks` for the ablation |
| `eval_seeds` | 4 | **not used by any code** |
| `seed` | 4242 | a single training seed |

---

## 5. Using the trained agent (`PPOPolicy`)

```python
def decide(self, state, legal_actions, sim=None):
    self.env.sim = sim                              # point the wrapper at the caller's world
    obs = self.env._observation()                   # the same 40-number layout as in training
    masks = self.env.action_masks() if self.use_masks else None
    index, _ = self.model.predict(obs, action_masks=masks, deterministic=True)   # argmax
    action = self.actions[index]
    if str(action) not in {str(a) for a in legal_actions}:       # only possible unmasked
        return Decision(WAIT (or first legal), f"PPO chose {action}, which is not possible now; waiting")
    probabilities = self._probabilities(obs, masks)
    return Decision(action, f"PPO policy (probability {p:.2f})", probabilities)
```

- **Deterministic at evaluation:** the most probable action is taken. Stochastic sampling was only for exploration during training.
- **Live:** `sim` is a `LiveState` (file 14). `_observation()` only needs `state()`, `p`, `matrix` and `shift_duration_s`, so the same code works on the robot.
- **Explanation:** only the action probability. That is "the most a policy network can say about why" (docstring). It is honest, but it is not an explanation in the sense of file 10.
- **The environment inside `PPOPolicy`** is `FactoryEnv(config_dir=None)`, the *balanced* scenario. It is used only for the action list and observation layout, which are the same in all scenarios (the charge targets come from `params.yaml`).
- Without a trained model it **refuses to run** (`FileNotFoundError`), rather than acting randomly.

---

## 6. Problems that affect the comparison (verified)

### 6.1 Bug: PPO was trained on only 4 of the 6 scenarios

```python
env = FactoryEnv(scenario=scenarios[rank % len(scenarios)], ...)   # rank = 0..n_envs-1 = 0..3
```

With `n_envs = 4`, only `scenarios[0..3]` are ever used, and each env keeps its scenario for all of training (`FactoryEnv.reset` only changes the seed). The training report lists the scenarios as `balanced, fault_burst, high_demand, low_battery_start, one_hot_section, worn_robot`, sorted. **PPO trained on `balanced`, `fault_burst`, `high_demand`, `low_battery_start` only.** It never saw `one_hot_section` or `worn_robot`, yet it is evaluated on them next to a neuro-symbolic model whose data covered all 6.

With today's 12 scenario files, a retrain would use `aged_battery`, `balanced`, `fault_burst`, `heavy_load` only.

**Fix:** pick a random scenario at every `reset()` (a wrapper holding the list), or set `n_envs = len(scenarios)`.

### 6.2 Unequal simulation budget

- PPO: **300 000** simulated actions.
- Neuro-symbolic training: about 13 500 labelled candidates per round × 3 futures × up to 900 s of look-ahead (≈ 14 actions at ≈ 62 s each) is roughly **570 000 simulated actions per round**, so about **1.7 million over 3 rounds**, plus the walking itself.

That is an estimate from the code and the logged sample counts, not a measured counter. Even so, it is clear that **PPO got several times less simulation experience**. A fair comparison should report both methods at **equal simulator budgets** (e.g. learning curves of score vs simulated actions).

### 6.3 One training seed, no learning curve

Deep-RL results vary strongly between random seeds (Henderson et al., "Deep RL that matters", 2018). One run at seed 4242 cannot say whether PPO is systematically better or worse. Standard practice is at least **3–5 training seeds**, reporting mean and spread, plus a learning curve.

### 6.4 Per-step discounting ignores action duration

`gamma = 0.995` per decision treats a 60 s WAIT and a 50-minute CHARGE the same. The neuro-symbolic training discounts by elapsed seconds. Because 0.995^60 ≈ 0.74 over a whole shift, the effect is modest here, but it is a methodological difference. See file 08 §2.2.

### 6.5 Observation gaps

PPO does not see absolute rates, unit mass, fault time remaining or cargo count (file 08 §6.3). One policy must serve all scenarios from an observation that does not fully identify the scenario.

### 6.6 No reward or observation normalisation

Rewards per step range from about −0.5 to +10 (a big delivery), and more with losses. SB3 recommends `VecNormalize` for reward scaling in many tasks. It was not used. It may not matter here, but it was not tested.

### 6.7 The unmasked ablation was never trained

`evaluate.py` offers `ppo_unmasked`, but `models/ppo_policy_unmasked.zip` does not exist, so selecting it raises `FileNotFoundError`. HANDOVER lists this as open.

### 6.8 Safety is only a penalty

PPO optimises the objective, where a safety violation costs −20 **once per deduplicated violation** (file 08 §3.5). In scenarios where extra throughput is worth more than 20 units, **the optimal policy under that reward violates safety**. The fresh evaluation (file 13) shows exactly this in `heavy_load`: PPO has the highest score (71.5) with violations in 9 of 10 episodes, a minimum battery of 7 % on average and 0.8 % in one episode. That is not PPO "misbehaving". It is PPO correctly optimising a reward that underprices safety. It is the strongest argument for **hard constraints** (the neuro-symbolic shield) or **constrained RL**.

---

## 7. What the results say about PPO (summary; details in file 13)

From HANDOVER WP5 (20 seeds, the 6 original scenarios):
- Score is close to the neuro-symbolic model (within about 1–3 points everywhere). PPO was better than symbolic-only in `balanced` and `one_hot_section`, and lower in `low_battery_start` (48.4 vs 54.0).
- Safety violations occurred in **5 of 20** `low_battery_start` episodes.
- Highest energy use: 10.1 Wh in balanced (neuro-symbolic 8.0, rule 4.9).

From the fresh run (10 seeds, 12 scenarios): PPO is competitive everywhere, best in `heavy_load` by score, with violations in `aged_battery` (2/10), `low_battery_start` (3/10) and `heavy_load` (9/10). It has the highest energy in almost every scenario (≈ 10 Wh vs 7–8 Wh for the neuro-symbolic model), with almost no charging.

---

## 8. Key concepts of this section

- Return, discount, value, action-value, advantage
- Policy gradient theorem, variance reduction with baselines
- Actor-critic architecture
- TD error, GAE(γ, λ)
- Trust-region idea, PPO clipped surrogate objective, value loss, entropy bonus
- On-policy vs off-policy learning; sample efficiency
- Invalid action masking
- Vectorised environments, `Monitor`, deterministic vs stochastic evaluation
- Reward hacking / specification gaming (safety as a soft penalty)
- Reproducibility of deep RL (seed variance)

---

## 9. Improvements and technologies for this section

| # | Idea | Why | How |
|---|---|---|---|
| 1 | **Fix scenario coverage** | PPO never saw 2 of its 6 evaluation scenarios | A `RandomScenarioEnv` wrapper that picks a scenario (and optionally randomised parameters) at every `reset()` |
| 2 | **Multiple seeds + learning curves at equal budget** | One seed; unequal budgets | Train 5 seeds; `EvalCallback` every 20 k steps on fixed evaluation seeds; plot score vs simulated actions for PPO **and** the neuro-symbolic pipeline |
| 3 | **Constrained RL** | Safety as a −20 penalty is exploitable | **PPO-Lagrangian** (e.g. OmniSafe or `safety-gymnasium` style): a cost signal (seconds below reserve, overheat) with a budget, where a Lagrange multiplier adapts the penalty until the constraint holds |
| 4 | **Shielded PPO** | The fairest "RL + rules" comparison: same shield, learned ranking | Apply the symbolic hard rules as an extra action mask during training *and* evaluation (`action_masks = legal ∧ allowed`); compare with the neuro-symbolic model to isolate the value of the ranking method |
| 5 | **SMDP-aware discounting** | Variable durations | Custom rollout buffer with per-transition γ^(Δt/τ) (SB3 needs a subclass), or options-style algorithms |
| 6 | **Richer observation / candidate-set policy** | The PPO input misses scenario-identifying quantities | Add the missing fields; or a policy that scores per-candidate feature vectors (same 27 features) with a shared network, so PPO and neuro-symbolic see identical information |
| 7 | **Hyperparameter tuning** | Defaults were used | `Optuna` with `rl-baselines3-zoo` style search over lr, n_steps, ent_coef, γ, λ, net size |
| 8 | **Recurrent PPO** | Partial observability (hidden fault/cycle state) | `sb3_contrib.RecurrentPPO` with an LSTM policy, which can remember recent failures and trends |
| 9 | **Off-policy / value-based alternatives** | Sample efficiency | Masked **DQN / QR-DQN** (discrete actions, replay buffer), or **offline RL** (CQL/IQL with `d3rlpy`) from logged rule and neuro-symbolic episodes |
| 10 | **Model-based RL / planning** | The simulator is known and cloneable, which pure model-free RL does not exploit | MuZero/AlphaZero-style MCTS with a learned value, or simply MCTS with the true simulator at decision time |
| 11 | **Train the unmasked ablation** | It is referenced but missing | `train_ppo.py --no-masks`; report how much of PPO's performance comes from the mask |
| 12 | **Reward/observation normalisation check** | Untested | `VecNormalize(norm_obs=False, norm_reward=True)`; compare learning curves |
