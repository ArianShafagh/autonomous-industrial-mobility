#!/usr/bin/env python3
"""Reads and checks the starter config (config/run.yaml) for scripts/run.sh.

    run_config.py load FILE [key=value ...]   print validated values as shell assignments
    run_config.py choices                     print scenarios / models / planners, one line each
    run_config.py save FILE key=value ...     write values back into FILE, keeping its comments

The valid choices are read from the workspace itself (the scenario files, the planner list in
navigation.launch.py), so a new scenario or planner shows up here without editing this script.
"""
import ast
import os
import re
import shlex
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIO_DIR = os.path.join(WS, "src", "robofetch_factory", "config", "scenarios")
NAV_LAUNCH = os.path.join(WS, "src", "robofetch_nav", "launch", "navigation.launch.py")
MODELS = ["ns", "ns_symbolic_only", "ppo", "rule", "fallback"]
VIEWS = ["gui", "headless"]
KEYS = ["ask", "scenario", "model", "planner", "view", "shift_minutes", "seed", "dashboard", "build"]


def scenarios():
    return sorted(f[:-5] for f in os.listdir(SCENARIO_DIR) if f.endswith(".yaml"))


def planners():
    """The PLANNERS tuple from navigation.launch.py, read without importing ROS."""
    with open(NAV_LAUNCH) as fh:
        match = re.search(r"^PLANNERS\s*=\s*(\(.*?\))", fh.read(), re.M | re.S)
    return list(ast.literal_eval(match.group(1)))


def as_bool(key, value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("true", "yes", "1", "on"):
        return True
    if text in ("false", "no", "0", "off"):
        return False
    raise ValueError(f"{key}: '{value}' is not true/false")


def as_int(key, value, minimum):
    try:
        number = int(str(value).strip())
    except ValueError:
        raise ValueError(f"{key}: '{value}' is not a whole number") from None
    if number < minimum:
        raise ValueError(f"{key}: {number} is below {minimum}")
    return number


def one_of(key, value, choices):
    value = str(value).strip()
    if value not in choices:
        raise ValueError(f"{key}: '{value}' is not one of: {', '.join(choices)}")
    return value


def validate(raw):
    unknown = sorted(set(raw) - set(KEYS))
    if unknown:
        raise ValueError(f"unknown key(s) {', '.join(unknown)}; valid keys: {', '.join(KEYS)}")
    missing = [k for k in KEYS if k not in raw]
    if missing:
        raise ValueError(f"missing key(s): {', '.join(missing)}")
    return {
        "ask": as_bool("ask", raw["ask"]),
        "scenario": one_of("scenario", raw["scenario"], scenarios()),
        "model": one_of("model", raw["model"], MODELS),
        "planner": one_of("planner", raw["planner"], planners()),
        "view": one_of("view", raw["view"], VIEWS),
        "shift_minutes": as_int("shift_minutes", raw["shift_minutes"], 0),
        "seed": as_int("seed", raw["seed"], -1),
        "dashboard": as_bool("dashboard", raw["dashboard"]),
        "build": as_bool("build", raw["build"]),
    }


def overrides(pairs):
    out = {}
    for pair in pairs:
        key, sep, value = pair.partition("=")
        if not sep or key not in KEYS:
            raise ValueError(f"bad override '{pair}'; use key=value with a key from: {', '.join(KEYS)}")
        out[key] = value
    return out


def load(path, pairs):
    with open(path) as fh:
        raw = yaml.safe_load(fh) or {}
    raw.update(overrides(pairs))
    cfg = validate(raw)
    for key, value in cfg.items():
        value = str(value).lower() if isinstance(value, bool) else str(value)
        print(f"CFG_{key.upper()}={shlex.quote(value)}")


def save(path, pairs):
    """Replace only the value on each `key: value` line, so comments and layout survive."""
    new = overrides(pairs)
    with open(path) as fh:
        text = fh.read()
    for key, value in new.items():
        def swap(m, value=value):   # keep the comment in its column when the value length changes
            pad = max(1, len(m.group(2)) + len(m.group(3)) - len(value)) if m.group(3) else 0
            return m.group(1) + value + " " * pad
        text, n = re.subn(rf"^({re.escape(key)}:[ \t]*)(\S+)([ \t]*)", swap, text, count=1,
                          flags=re.M)
        if n == 0:
            text += f"{key}: {value}\n"
    validate(yaml.safe_load(text))      # never write a file that would not load
    with open(path, "w") as fh:
        fh.write(text)


def main(argv):
    try:
        if argv[:1] == ["load"] and len(argv) >= 2:
            load(argv[1], argv[2:])
        elif argv[:1] == ["save"] and len(argv) >= 3:
            save(argv[1], argv[2:])
        elif argv[:1] == ["choices"]:
            print("SCENARIOS=" + shlex.quote(" ".join(scenarios())))
            print("MODELS=" + shlex.quote(" ".join(MODELS)))
            print("PLANNERS=" + shlex.quote(" ".join(planners())))
        else:
            print(__doc__, file=sys.stderr)
            return 2
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"[run] config error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
