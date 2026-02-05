# Source Code

ROS2 packages and nodes for BFMC autonomous vehicle navigation system.

## Structure
```
src/
├── bfmc_perception/           Perception and detection nodes
│   ├── bfmc_perception/
│   │   ├── traffic_detector.py
│   │   ├── lane_detector.py
│   │   └── sensor_fusion.py
│   ├── package.xml
│   └── setup.py
├── bfmc_control/              Control and FSM nodes
│   ├── bfmc_control/
│   │   ├── fsm_controller.py
│   │   ├── safety_monitor.py
│   │   └── pid_controller.py
│   ├── package.xml
│   └── setup.py
├── bfmc_planning/             Path planning nodes
│   ├── bfmc_planning/
│   │   ├── global_planner.py
│   │   ├── local_planner.py
│   │   └── dwa_planner.py
│   ├── package.xml
│   └── setup.py
└── bfmc_msgs/                 Custom message definitions
    ├── msg/
    │   ├── TrafficSign.msg
    │   ├── FSMState.msg
    │   └── LaneInfo.msg
    ├── CMakeLists.txt
    └── package.xml
```

## Packages Overview

### bfmc_perception
Perception pipeline for traffic sign detection, lane detection, and sensor fusion.

**Nodes:**

**traffic_detector**
- Real-time traffic sign detection using CNN
- Detection rate: 20 Hz
- Accuracy: 92%
- Supports 12 traffic sign classes

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image)

**Published Topics:**
- `/traffic_signs` (bfmc_msgs/TrafficSign)
- `/detection_image` (sensor_msgs/Image) - Debug visualization

**Parameters:**
- `model_path`: Path to trained model
- `confidence_threshold`: Detection confidence (default: 0.7)
- `detection_rate`: Hz (default: 20.0)

**lane_detector**
- Lane detection and tracking
- ROI-based processing for efficiency
- Outputs lane center and curvature

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image)

**Published Topics:**
- `/lane_info` (bfmc_msgs/LaneInfo)

**sensor_fusion**
- Fuses camera, LiDAR, and IMU data
- EKF-based state estimation
- Provides unified perception output

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image)
- `/scan` (sensor_msgs/LaserScan)
- `/imu` (sensor_msgs/Imu)

**Published Topics:**
- `/fused_perception` (custom message)

---

### bfmc_control
Control system including FSM controller and safety monitoring.

**Nodes:**

**fsm_controller**
- Finite state machine for traffic rule compliance
- 50 Hz control loop
- State-based velocity commands

**States:**
1. **CRUISE** - Normal navigation at target speed
2. **APPROACHING_STOP** - Deceleration sequence
3. **STOPPED** - Zero velocity (3 second duration)
4. **YIELD** - Speed reduction for yield signs
5. **EMERGENCY_BRAKE** - Immediate stop for obstacles

**Subscribed Topics:**
- `/traffic_signs` (bfmc_msgs/TrafficSign)
- `/scan` (sensor_msgs/LaserScan)
- `/emergency_stop` (std_msgs/Bool)

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist)
- `/fsm_state` (bfmc_msgs/FSMState)

**Parameters:**
- `max_speed`: Maximum linear velocity (default: 0.6 m/s)
- `control_frequency`: Hz (default: 50.0)
- `stop_duration`: Seconds at stop sign (default: 3.0)
- `stop_distance`: Distance to stop line (default: 0.5m)

**safety_monitor**
- Emergency stop and collision avoidance
- LiDAR-based obstacle detection
- Highest priority override

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan)

**Published Topics:**
- `/emergency_stop` (std_msgs/Bool)

**Parameters:**
- `emergency_stop_distance`: Critical distance (default: 0.3m)
- `warning_distance`: Warning threshold (default: 0.6m)

**pid_controller**
- Low-level velocity control
- PID tuning for smooth motion

---

### bfmc_planning
Path planning stack for global and local navigation.

**Nodes:**

**global_planner**
- A* algorithm for global path planning
- Occupancy grid-based

**local_planner**
- Dynamic Window Approach (DWA)
- Real-time obstacle avoidance
- Cost function: goal alignment + obstacle clearance

**dwa_planner**
- Velocity space sampling
- Dynamic constraints consideration

---

### bfmc_msgs
Custom ROS2 message definitions.

**Message Types:**

**TrafficSign.msg**
```
uint8 STOP = 0
uint8 YIELD = 1
uint8 SPEED_30 = 2
uint8 SPEED_50 = 3
# ... (12 classes total)

uint8 sign_type
float32 confidence
float32 distance
geometry_msgs/Point position
```

**FSMState.msg**
```
uint8 CRUISE = 0
uint8 APPROACHING_STOP = 1
uint8 STOPPED = 2
uint8 YIELD = 3
uint8 EMERGENCY_BRAKE = 4

uint8 current_state
float32 time_in_state
```

**LaneInfo.msg**
```
float32 center_offset
float32 curvature
bool lane_detected
```

---

## Building Packages
```bash
# Navigate to workspace
cd ~/bfmc_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select bfmc_perception

# Build with symlink install (for Python development)
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Running Nodes
```bash
# Single node
ros2 run bfmc_perception traffic_detector

# With parameters
ros2 run bfmc_control fsm_controller --ros-args -p max_speed:=0.5

# Using launch file (recommended)
ros2 launch bfmc_nav bfmc_sim.launch.py
```

## Node Communication Flow
```
Camera → traffic_detector → TrafficSign → fsm_controller → cmd_vel → Robot
                                              ↑
LiDAR  → safety_monitor → emergency_stop ────┘
```

## Development

**Adding New Node:**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)
    
    def callback(self, msg):
        # Process data
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Adding to package:**
1. Place in `bfmc_perception/bfmc_perception/my_node.py`
2. Update `setup.py` entry points
3. Rebuild: `colcon build --packages-select bfmc_perception`

## Testing
```bash
# Unit tests
colcon test --packages-select bfmc_perception

# Integration tests
python scripts/testing/test_fsm.py

# Performance benchmarks
python scripts/testing/benchmark_performance.py
```

## Troubleshooting

**Import errors:**
```bash
# Rebuild and source
colcon build --symlink-install
source install/setup.bash
```

**Node not found:**
```bash
# Check package installation
ros2 pkg list | grep bfmc

# Verify executables
ros2 pkg executables bfmc_perception
```

**Topic not publishing:**
```bash
# List topics
ros2 topic list

# Check node status
ros2 node list

# Monitor topic
ros2 topic echo /traffic_signs
```

## Performance Targets

- Traffic Detection: 20 Hz
- FSM Control: 50 Hz
- Lane Detection: 30 Hz
- Sensor Fusion: 10 Hz
- Safety Monitor: 50 Hz

## Notes

- All nodes use ROS2 parameters for configuration
- Perception nodes support both simulation and hardware
- FSM has priority system (safety > traffic > navigation)
- Custom messages defined in bfmc_msgs package
