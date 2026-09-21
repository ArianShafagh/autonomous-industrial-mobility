# Setting the project up on a new PC

Everything needed is in two places:

1. **This GitHub repo** — all code, configs, trained models (`src/robofetch_ai/robofetch_ai/models/`), and `HANDOVER.md` (the full work log with every result).
2. **The transfer archive** `thesis_transfer_<date>.tar.gz` — what git deliberately ignores: result CSVs and figures (`tools/ai/results/`), the WP6 PPO checkpoint (`tools/ai/checkpoints/`), planner results (`tools/nav/results/`), Gazebo run logs (`logs/`), the `docs/` folder, and the Claude Code memory notes.

The venv, `build/` and `install/` are **not** copied; they are rebuilt (steps 3–5).

## 1. System (Ubuntu 24.04, or Windows with WSL2 + Ubuntu 24.04)

```bash
# ROS 2 Jazzy: follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz ros-jazzy-nav2-smac-planner ros-jazzy-nav2-theta-star-planner \
    python3-colcon-common-extensions python3-venv git
```

Gazebo Harmonic comes with `ros-jazzy-ros-gz`. Check: `gz sim --version` (8.x).

## 2. Code

```bash
mkdir -p ~/robofetch_ws_copy && cd ~/robofetch_ws_copy
git clone git@github.com:ArianShafagh/autonomous-industrial-mobility.git robofetch_ws
cd robofetch_ws
git config user.name  "ArianShafagh"
git config user.email "arian.shafagh2003@gmail.com"
```

The repo is private: the new PC needs an SSH key added on GitHub (`ssh-keygen -t ed25519`, then paste `~/.ssh/id_ed25519.pub` at github.com → Settings → SSH keys), or clone over HTTPS with a personal access token.

Keeping the same path (`~/robofetch_ws_copy/robofetch_ws`) is not required, but it matches the logs and HANDOVER.

## 3. Python environment

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages venv      # system site-packages: rclpy comes from ROS
venv/bin/pip install -r requirements.txt         # numpy<2 and setuptools<80 are pinned on purpose
touch venv/COLCON_IGNORE
```

## 4. Build

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

## 5. Restore the transfer archive

Copy `thesis_transfer_<date>.tar.gz` into the workspace root, then:

```bash
tar -xzf thesis_transfer_<date>.tar.gz          # restores tools/ai/results, logs, docs, ...
# Claude Code memory (optional, lets Claude pick up the project rules):
mkdir -p ~/.claude/projects/-home-$USER-robofetch_ws_copy-robofetch_ws/memory
cp -r claude_memory/* ~/.claude/projects/-home-$USER-robofetch_ws_copy-robofetch_ws/memory/
```

(The memory folder name is the workspace path with `/` replaced by `-`; adjust it if the workspace lives elsewhere.)

## 6. Check that everything works

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
venv/bin/python -m pytest -q src/robofetch_core/test src/robofetch_factory/test src/robofetch_ai/test
#   -> 163 passed
venv/bin/python -u tools/ai/evaluate.py --policies rule ns --scenarios balanced --seeds 3
#   -> fast simulator runs, ns scores ~49
./scripts/run.sh --dry-run --yes
#   -> prints the launch command
./scripts/run.sh                                  # full system: Gazebo + Nav2 + AI + dashboard
```

The dashboard is on http://localhost:8000. Stop with Ctrl+C, then `./scripts/stop.sh`.

## Where things stand

See the end of `HANDOVER.md`: WP0–WP9 and the WP9 follow-up are done and committed. What remains is writing the thesis, plus the optional open issues listed there.
