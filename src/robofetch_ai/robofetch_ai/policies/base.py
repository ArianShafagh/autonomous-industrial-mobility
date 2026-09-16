"""What every decision model must provide.

One interface for all of them — rule-based reference, neuro-symbolic, PPO — so the fast
simulator, the evaluation runs and the live decision service can use any of them unchanged.

A policy sees the same facts the live system publishes (`FactorySim.state()` mirrors
`/factory/<id>/status` and `/robot/telemetry`), is given the legal actions, and returns ONE
action plus a human-readable reason. The reason is not decoration: it is what the dashboard
shows and what the thesis reports as the explanation of a decision.
"""
from dataclasses import dataclass, field


@dataclass
class Decision:
    action: object                      # mission_plan.Action
    explanation: str = ""               # why this action, in words
    scores: dict = field(default_factory=dict)   # score per candidate action, for analysis
    latency_ms: float = 0.0


class Policy:
    name = "policy"

    def reset(self):
        """Called at the start of every shift (episode)."""

    def decide(self, state, legal_actions, sim=None):
        """Choose one of `legal_actions`. `sim` is available for look-ahead; may be None."""
        raise NotImplementedError

    def __str__(self):
        return self.name
