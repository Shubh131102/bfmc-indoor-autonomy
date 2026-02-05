# Launch Files

ROS2 launch files for BFMC autonomous vehicle system.

## Available Launch Files

### bfmc_sim.launch.py
**Purpose:** Complete simulation environment with all navigation components

**Launched Nodes:**
- `traffic_detector`: Real-time traffic sign detection (20 Hz)
- `lane_detector`: Lane detection and tracking
- `sensor_fusion`: Multi-sensor data integration
- `fsm_controller`: Finite state machine control (50 Hz)
- `path_planner`: Path planning (A* + DWA)
- `safety_monitor`: Emergency stop and collision avoidance

**Arguments:**
```bash
use_sim_time    Use simulation time (default: true)
world           Gazebo world name (default: bfmc_track)
camera_topic    Camera topic (default: /camera/image_raw)
max_speed       Maximum speed in m/s (default: 0.6)
```

**Usage:**
```bash
# Basic launch
ros2 launch bfmc_nav bfmc_sim.launch.py

# Custom parameters
ros2 launch bfmc_nav bfmc_sim.launch.py max_speed:=0.8 camera_topic:=/front_camera/image
```

---

### navigation.launch.py
**Purpose:** Navigation stack only (no simulation)

**Usage:**
```bash
ros2 launch bfmc_nav navigation.launch.py
```

---

### traffic_detection.launch.py
**Purpose:** Traffic sign detection node only

**Usage:**
```bash
ros2 launch bfmc_nav traffic_detection.launch.py model_path:=models/yolo.pth
```

---

### hardware.launch.py
**Purpose:** Launch for Raspberry Pi hardware deployment

**Usage:**
```bash
ros2 launch bfmc_nav hardware.launch.py
```

## Node Communication
```
/camera/image_raw (sensor_msgs/Image)
    └─> traffic_detector
        └─> /traffic_signs (custom_msgs/TrafficSign)
            └─> fsm_controller
                └─> /cmd_vel (geometry_msgs/Twist)

/scan (sensor_msgs/LaserScan)
    └─> safety_monitor
        └─> /emergency_stop (std_msgs/Bool)
            └─> fsm_controller
```

## FSM States

The FSM controller manages these states:
- **CRUISE:** Normal navigation at target speed
- **APPROACHING_STOP:** Decelerating for stop sign
- **STOPPED:** Zero velocity at stop line (3s duration)
- **YIELD:** Speed reduction for yield sign
- **EMERGENCY_BRAKE:** Immediate stop for obstacles

## Common Patterns

**Launch with specific configuration:**
```bash
ros2 launch bfmc_nav bfmc_sim.launch.py \
  max_speed:=0.5 \
  camera_topic:=/camera/image_raw \
  use_sim_time:=true
```

**Launch with RViz:**
```bash
# Edit launch file to uncomment rviz_node
ros2 launch bfmc_nav bfmc_sim.launch.py
```

**Monitor topics:**
```bash
# List active topics
ros2 topic list

# Monitor traffic detection
ros2 topic echo /traffic_signs

# Monitor control commands
ros2 topic echo /cmd_vel
```

## Troubleshooting

**Camera not found:**
```bash
# Check available cameras
ros2 topic list | grep camera

# Verify camera node
ros2 node list | grep camera
```

**FSM not responding:**
```bash
# Check FSM state
ros2 topic echo /fsm_state

# Verify traffic sign detection
ros2 topic echo /traffic_signs
```

**Nodes not launching:**
```bash
# Check package installation
ros2 pkg list | grep bfmc

# Rebuild workspace
colcon build --packages-select bfmc_nav
```

## Notes

- All nodes use `use_sim_time` parameter for simulation synchronization
- Traffic detection runs at 20 Hz for real-time performance
- FSM control loop runs at 50 Hz for responsive behavior
- Safety monitor has highest priority (can override FSM commands)
