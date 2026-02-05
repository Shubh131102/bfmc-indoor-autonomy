# Configuration Files

ROS2 parameter files and system configuration for BFMC autonomous vehicle.

## Structure
```
config/
├── navigation_params.yaml     Navigation stack parameters
├── camera_params.yaml         Camera calibration and settings
├── traffic_detection.yaml     Traffic sign detection parameters
├── fsm_config.yaml           Finite state machine configuration
└── vehicle_params.yaml        TurtleBot3/vehicle-specific parameters
```

## Configuration Files

### navigation_params.yaml
Navigation and control parameters:
- Maximum linear velocity
- Maximum angular velocity
- Control loop frequency
- Safety thresholds

### camera_params.yaml
Camera configuration:
- Resolution and framerate
- Camera matrix and distortion coefficients
- Image preprocessing parameters
- ROI for traffic sign detection

### traffic_detection.yaml
Traffic sign detection model parameters:
- Model path and architecture
- Confidence threshold
- NMS threshold
- Detection rate (20 Hz)
- Supported sign classes (12 types)

### fsm_config.yaml
Finite state machine states and transitions:
- State definitions (CRUISE, STOP, YIELD, etc.)
- Transition conditions
- Timeout parameters
- Safety override conditions

### vehicle_params.yaml
Platform-specific parameters:
- Wheel radius and base
- Maximum speeds
- Sensor positions and transforms

## Usage

Configuration files are loaded automatically by launch files:
```bash
ros2 launch bfmc_nav navigation.launch.py config:=config/navigation_params.yaml
```

## Notes

All parameters use SI units (meters, radians, seconds) unless otherwise specified.
