# BFMC – Indoor Autonomous Navigation (ROS2 + Raspberry Pi)

**TL;DR:** ROS2-based indoor autonomy stack for the Bosch Future Mobility Challenge (BFMC) platform with computer vision perception, a rule-based FSM, and a deployment dashboard for Raspberry Pi.

**Results (from project work):**
- ~**15% faster** simulation → hardware transfer time (smoother bring-up & validation)
- **85%** reduction in rule violations and **+30%** destination success in sim before hardware transfer

## What’s here
- `src/` — ROS2 packages (nodes for perception, planning, control)
- `launch/` — launch files for sim and hardware
- `config/` — YAML configs (camera, controller, topics, parameters)
- `scripts/` — utilities (dataset capture, deployment helpers)
- `docker/` — Dockerfile + compose for reproducible setup (optional)
- `media/` — demos (gif/mp4)
- `docs/` — diagrams, notes, and reports

## Quick Start (dev)
```bash
# Python deps (non-ROS) — adjust to your environment
python -m venv .venv
# Windows:
.\.venv\Scripts\activate
pip install -r requirements.txt

# ROS2 workspace (example)
# colcon build
# source install/setup.bash     # or setup.bat on Windows, or via WSL
# ros2 launch <pkg> bfmc_sim.launch.py
