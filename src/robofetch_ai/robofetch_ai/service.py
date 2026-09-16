"""The decision service: the robot asks "what next?", a model answers.

    POST /decide   {model, scenario, state}   -> {action, explanation, scores, latency_ms, model}
    GET  /health                              -> which models are loaded and ready
    GET  /models                              -> the available decision models

It runs as a SEPARATE process on its own port, exactly as in the old project, and for the same
reason: "the AI is unavailable" has to be a state you can produce by stopping a process, so the
robot's fallback behaviour is testable rather than theoretical. The mission executor falls back to
its own built-in rule when this service does not answer.

The models here are the ones evaluated in the fast simulator - same code, same weights - so what
the thesis measures offline is what drives the robot.

    venv/bin/python -m uvicorn robofetch_ai.service:app --port 8001
"""
import math
import os
import time
import traceback

from fastapi import FastAPI
from pydantic import BaseModel

from robofetch_ai.env.live_state import LiveState

DEFAULT_SCENARIO = os.environ.get("ROBOFETCH_SCENARIO", "balanced")
MODELS = ("ns", "ns_symbolic_only", "ns_neural_only", "ppo", "rule")

app = FastAPI(title="RoboFetch decision service")
_contexts = {}          # scenario -> LiveState
_policies = {}          # (model, scenario) -> Policy
_stats = {"decisions": 0, "failures": 0}


class DecideRequest(BaseModel):
    state: dict
    model: str = "ns"
    scenario: str = DEFAULT_SCENARIO


def context(scenario):
    if scenario not in _contexts:
        _contexts[scenario] = LiveState(scenario)
    return _contexts[scenario]


def policy(name, scenario):
    """Built once and kept: loading a network on every request would cost more than deciding."""
    key = (name, scenario)
    if key not in _policies:
        cfg = context(scenario).cfg
        if name == "rule":
            from robofetch_ai.policies.rule_based import RuleBasedPolicy
            _policies[key] = RuleBasedPolicy()
        elif name.startswith("ns"):
            from robofetch_ai.policies.neurosymbolic import NeuroSymbolicPolicy
            mode = {"ns": "full", "ns_neural_only": "neural",
                    "ns_symbolic_only": "symbolic"}[name]
            _policies[key] = NeuroSymbolicPolicy(cfg, mode=mode)
        elif name == "ppo":
            from robofetch_ai.policies.rl_ppo import PPOPolicy
            _policies[key] = PPOPolicy(cfg)
        else:
            raise ValueError(f"unknown model '{name}', known: {', '.join(MODELS)}")
    return _policies[key]


def json_safe(value):
    """JSON has no infinity or NaN; a decision must still be reportable."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


@app.get("/health")
def health():
    return {"ok": True, "loaded": [f"{m}/{s}" for m, s in _policies],
            "decisions": _stats["decisions"], "failures": _stats["failures"],
            "models": list(MODELS)}


@app.get("/models")
def models():
    return {"models": list(MODELS)}


@app.post("/decide")
def decide(request: DecideRequest):
    started = time.perf_counter()
    try:
        live = context(request.scenario)
        state = dict(request.state)
        # The caller sends `null` where a value is infinite (time to full of a stopped line).
        for section in state.get("sections", {}).values():
            if section.get("time_to_full_s") is None:
                section["time_to_full_s"] = math.inf
            if section.get("fault_remaining_s") is None:
                section["fault_remaining_s"] = 0.0
        live.update(**state)
        chosen = policy(request.model, request.scenario).decide(
            live.state(), live.legal_actions(), live)
        _stats["decisions"] += 1
        return {
            "action": str(chosen.action),
            "explanation": chosen.explanation,
            "scores": json_safe(chosen.scores),
            "model": request.model,
            "latency_ms": round((time.perf_counter() - started) * 1000.0, 2),
        }
    except Exception as exc:                                         # noqa: BLE001
        # A failing model must not take the robot down with it: report the failure and let the
        # executor use its fallback.
        _stats["failures"] += 1
        return {"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc(),
                "model": request.model,
                "latency_ms": round((time.perf_counter() - started) * 1000.0, 2)}


def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("ROBOFETCH_AI_PORT", 8001)))


if __name__ == "__main__":
    main()
