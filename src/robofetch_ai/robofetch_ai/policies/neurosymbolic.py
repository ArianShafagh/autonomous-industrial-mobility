"""The neuro-symbolic decision model — the thesis's contribution.

    legal actions
        │
        ├─ SYMBOLIC hard rules ──► forbidden actions are dropped (safety, never overridable)
        │
        ├─ SYMBOLIC priority rules ──► keep only the most urgent tier
        │      tier 2 unblock a stopped line · tier 1 prevent one from stopping · tier 0 routine
        │
        └─ NEURAL scorer ──► ranks what is left by expected value
                │
                └─► decision + explanation ("B is BLOCKED ...; chosen over DELIVER by 0.42")

Three modes, so the thesis can show what each half contributes:
    full      rules + network            (the model)
    neural    network alone              (no safety rules, no priorities: what the net learnt)
    symbolic  rules alone                (priority tier, then the cheapest action per unit)

If no trained network is available the model runs in `symbolic` mode and says so, which is also
what the live service falls back to.
"""
import math
import time

from robofetch_core.mission_plan import CHARGE, DELIVER, PICKUP, WAIT
from robofetch_core.robot_model import battery_percent_for, trip_energy_wh

from robofetch_ai.policies.base import Decision, Policy
from robofetch_ai.policies.features import action_features, symbolic_estimate
from robofetch_ai.policies.neural import DEFAULT_MODEL, ActionScorer
from robofetch_ai.policies.symbolic import TIER_NAMES, SymbolicLayer


class NeuroSymbolicPolicy(Policy):
    def __init__(self, cfg, mode="full", model_path=DEFAULT_MODEL, scorer=None):
        if mode not in ("full", "neural", "symbolic"):
            raise ValueError(f"unknown mode '{mode}' (full, neural, symbolic)")
        self.cfg = cfg
        self.mode = mode
        self.rules = SymbolicLayer(cfg)
        self.scorer = scorer
        self.model_note = ""
        if mode in ("full", "neural") and scorer is None:
            try:
                self.scorer = ActionScorer.load(model_path)
            except (FileNotFoundError, ValueError) as exc:
                self.scorer = None
                self.model_note = f"no trained scorer ({exc}); running on rules alone"
                self.mode = "symbolic"
        self.name = {"full": "ns", "neural": "ns_neural_only", "symbolic": "ns_symbolic_only"}[mode]

    # ------------------------------------------------------------------------------ helpers
    def _neural_scores(self, state, actions, p, matrix):
        """Symbolic estimate + the network's learnt correction (see features.symbolic_estimate)."""
        rows = [action_features(state, a, p, matrix) for a in actions]
        corrections = self.scorer.score(rows)
        objective = self.cfg["mission"]["objective"]
        return {a: symbolic_estimate(state, a, p, matrix, objective) + c
                for a, c in zip(actions, corrections)}

    @staticmethod
    def _cost_per_unit(state, action, p, matrix):
        """Energy per unit this action would move - the symbolic-only ranking."""
        location, dest = state["location"], action.destination()
        distance = matrix[location][dest] if dest else 0.0
        energy = trip_energy_wh(p, distance, state["payload_kg"], state["temperature_c"],
                                state["condition_percent"])
        if action.kind == PICKUP:
            units = state["sections"][action.target]["units_that_fit"]
        elif action.kind == DELIVER:
            units = state["cargo_units"]
        else:
            units = 0
        return energy / units if units else float("inf")

    # -------------------------------------------------------------------------------- decide
    def decide(self, state, legal_actions, sim=None):
        started = time.perf_counter()
        p, matrix = sim.p, sim.matrix
        verdicts = self.rules.evaluate(state, legal_actions, p, matrix)

        if self.mode == "neural":
            # No rules at all: the network ranks every legal action.
            scores = self._neural_scores(state, list(legal_actions), p, matrix)
            action = max(scores, key=scores.get)
            return self._decision(action, f"neural score {scores[action]:+.2f} (no rules)",
                                  scores, started)

        candidates, tier = self.rules.allowed_by_tier(verdicts)
        if not candidates:
            # Every action is forbidden: the only safe thing left is to stand still, and to say
            # exactly why - this is the situation an operator must see.
            waits = [a for a in legal_actions if a.kind in (WAIT, CHARGE)]
            action = waits[0] if waits else list(legal_actions)[0]
            blocked = "; ".join(f"{a}: {v.why()}" for a, v in verdicts.items() if not v.allowed)
            return self._decision(action, f"every other action is forbidden ({blocked})",
                                  {}, started)

        if len(candidates) == 1:
            action = next(iter(candidates))
            return self._decision(action, self._why(action, verdicts, tier), {}, started)

        if self.mode == "symbolic":
            scores = {a: -self._cost_per_unit(state, a, p, matrix) for a in candidates}
            action = max(scores, key=scores.get)
            cost = -scores[action]
            extra = (f"cheapest at {cost:.3f} Wh per unit" if cost != float("inf")
                     else "no cheaper alternative")
            return self._decision(action, f"{self._why(action, verdicts, tier)}; {extra}",
                                  scores, started)

        scores = self._neural_scores(state, list(candidates), p, matrix)
        ranked = sorted(scores, key=scores.get, reverse=True)
        action = ranked[0]
        margin = ("" if len(ranked) < 2 else
                  f", chosen over {ranked[1]} by {scores[action] - scores[ranked[1]]:+.2f}")
        return self._decision(action,
                              f"{self._why(action, verdicts, tier)}; neural score "
                              f"{scores[action]:+.2f}{margin}", scores, started)

    def _why(self, action, verdicts, tier):
        reason = verdicts[action].why()
        label = TIER_NAMES.get(tier, "routine")
        refused = [f"{a} refused: {v.refusals[0]}" for a, v in verdicts.items() if not v.allowed]
        text = f"[{label}] {reason}"
        if refused:
            text += " | " + "; ".join(refused[:2])
        return text

    def _decision(self, action, explanation, scores, started):
        if self.model_note:
            explanation += f" | {self.model_note}"
        # "moves no units" scores as minus infinity internally; reported as None so the score can
        # travel over HTTP to the dashboard (JSON has no infinity).
        clean = {str(a): (round(float(s), 3) if math.isfinite(s) else None)
                 for a, s in scores.items()}
        return Decision(action, explanation, clean, (time.perf_counter() - started) * 1000.0)
