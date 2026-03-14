# BFMC — Autonomous Vehicle Navigation Stack

ROS2-based autonomous navigation stack for the Bosch Future Mobility Challenge 
(BFMC) platform — featuring CNN-based perception, traffic rule compliance, 
sim-to-real deployment, and a real-time telemetry dashboard.

---

## Key Results

- **85%** traffic rule compliance across highway, ramps, intersections, and pedestrian zones
- **92%** traffic sign and pedestrian detection accuracy at **20Hz**
- **0.8mm** sensor positional stability under dynamic vibration loading (FEA-validated mounts)
- Successful sim-to-real transfer from Gazebo to physical vehicle

---

## Stack Overview

| Component | Implementation |
|-----------|---------------|
| Perception | CNN-based traffic sign detection, LiDAR pedestrian detection |
| Sensor Mounts | SolidWorks + ANSYS FEA validated to 0.8mm positional stability |
| Navigation | ROS2 path planning with traffic rule compliance |
| Simulation | Gazebo with full track — highway, ramps, roundabout, bus lane |
| Deployment | Raspberry Pi with real-time telemetry dashboard |
| Sensors | Camera + LiDAR multi-sensor fusion |

---

## Repo Layout
```
bfmc_autonomous/
├── src/          # ROS2 packages — perception, planning, control
├── launch/       # Sim and hardware launch files
├── config/       # Camera, controller, topic parameters
├── scripts/      # Dataset capture and deployment utilities
├── docker/       # Dockerfile for reproducible setup
├── media/        # GIF and MP4 demos
└── docs/         # Diagrams, sensor mount CAD, reports
```

---

## Quick Start
```bash
# Python dependencies
python -m venv .venv
source .venv/bin/activate      # Linux/Mac
.\.venv\Scripts\activate       # Windows
pip install -r requirements.txt

# ROS2 workspace
colcon build
source install/setup.bash

# Launch simulation
ros2 launch bfmc_nav bfmc_sim.launch.py

# Launch on hardware
ros2 launch bfmc_nav bfmc_hardware.launch.py
```

---

## Traffic Rules Implemented

- Lane following — camera-based
- Traffic light detection — CNN color classification
- Stop sign compliance — detection + braking logic
- Pedestrian avoidance — LiDAR-based
- Shortest path navigation from start to destination

---

## Mechanical Design

Sensor mounting system designed and fabricated alongside software stack:
- LiDAR and camera mounts designed in SolidWorks
- ANSYS FEA under dynamic vibration loading — validated 0.8mm positional stability
- GD&T tolerancing on critical datum features for sub-millimeter alignment
- Laser cut components with DFA principles for cable routing and serviceability

---

## Telemetry Dashboard

Python-based real-time dashboard monitoring:
- Vehicle speed and steering angle
- Battery status
- Driving mode (manual/auto/legacy)
- Live camera feed
- CPU and memory utilization

Access: http://localhost:36187

---

## Troubleshooting

Perception not detecting signs:
```bash
ros2 topic echo /camera/image_raw    # Verify camera feed
ros2 topic hz /detections            # Check detection rate
```

LiDAR not publishing:
```bash
ros2 topic echo /scan
ros2 run tf2_tools view_frames
```

Hardware connection issues:
```bash
# Check Raspberry Pi connection
ping 192.168.1.1
ros2 topic list                      # Verify topics active
```

---

## Notes

- Test all perception nodes in simulation before hardware deployment
- Calibrate camera and LiDAR before each hardware session
- Sensor mount alignment critical — verify 0.8mm tolerance maintained
- Source workspace before running any ROS2 commands
