# HANDOVER — Thesis work log

**Thesis:** Towards Autonomous Industrial Mobility: An AI Framework for Optimal Path Planning and Resource-Efficient Navigation
**Base:** copy of the RoboFetch workspace (ROS 2 Jazzy, Gazebo Harmonic 8.11, Nav2 + MPPI + AMCL).
**Repository:** new private repo `github.com/ArianShafagh/autonomous-industrial-mobility`, fresh git history (branch `main`).
**Old project:** stays unchanged at `github.com/ArianShafagh/robofetch` (last commit `db4fc40`) and in `~/robofetch_ws`.

Each work package (WP) gets one section: what was done, files, decisions, problems, real results, open issues.

---

## WP0 — Clean slate (2026-09-15)

### Done
- Git: the old history was dropped from this workspace and a **new repository with a fresh history** was started (the project is a different project). The old repo on GitHub is untouched. The old local `.git` was moved out of the workspace, not pushed anywhere.
  The 12 "modified" scripts in the initial git status were **only lost executable bits** from the copy (0 line changes) — restored with `chmod +x`.
- Deleted (untracked, not recoverable, not needed): `build/`, `install/`, `log/`, `logs/` (≈110 MB), broken `robofetch_venv/` (365 MB), `robofetch.db`, `report.zip`, 905 `*:Zone.Identifier` files, all `__pycache__/`, `.pytest_cache/`, `.vscode/` caches.
- Deleted old-project files (still available in the old `robofetch` repo): `report/`, `tools/report/` (UML/figures/plantuml), `docs/` (proposal, diagrams, ARCHITECTURE), old `HANDOVER.md`, `README.md`, `INSTALL.md`, `scripts/acceptance.py`, `scripts/toggle_ai.sh`. 86 tracked files removed in total.
- New `requirements.txt`, rewritten `.gitignore` (ignores `venv/`, build dirs, `tools/ai/results/`, `tools/ai/checkpoints/`, caches).
- New venv `venv/` (924 MB) created with `--system-site-packages` so ROS's `rclpy` is importable.

### Decisions
- **numpy pinned `<2`**: ROS Jazzy system Python packages are built against numpy 1.x; mixing 2.x in the venv risks ABI errors.
- **setuptools pinned `<80`**: pip pulled setuptools 84, which conflicts with colcon-core 0.21 (`requires setuptools<80`) and breaks `ament_python` builds when the venv is active.
- **CPU PyTorch** (RTX 2070 exists, but the models are small MLPs; PPO with MLP policies is faster on CPU and the CPU wheel is ~10× smaller).

### Problems
- `rm -rf robofetch_venv` left a phantom `lib64` entry inside the Bash sandbox (a sandbox view artefact, not a real file); removing it outside the sandbox worked.

### Results / verification
```
venv: torch 2.14.0+cpu, gymnasium 1.3.0, stable-baselines3 2.9.0, numpy 1.26.4, setuptools 79.0.1, rclpy import OK
colcon build --symlink-install  ->  Summary: 10 packages finished [31.3s]
pytest src/robofetch_core/test  ->  6 passed in 0.03s   (robot_model.py still intact)
```

### Open issues (handled in later WPs)
- Old code still references removed things and will be rewritten/deleted in its WP: `delivery.launch.py` hard-codes `~/robofetch_ws/robofetch_venv/bin/python` (WP7), `scripts/order.sh`, `tools/ml/*`, bridge tests, gripper (WP1/WP3/WP7).
- Current workspace path is `~/robofetch_ws_copy/robofetch_ws`; launch/scripts must derive paths instead of hard-coding them (WP7).

### How to set up the environment
```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages venv
venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
source venv/bin/activate && colcon build --symlink-install && source install/setup.bash
```
