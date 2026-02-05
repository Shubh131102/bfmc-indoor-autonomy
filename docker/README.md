# Docker Environment

Containerized development environment for BFMC autonomous vehicle project.

## Prerequisites

- Docker installed
- Docker Compose installed (optional, for docker-compose workflow)
- X11 forwarding for GUI applications (RViz, RQT)

## Quick Start

### Option 1: Docker Compose (Recommended)
```bash
# Build image
docker-compose -f docker/docker-compose.yml build

# Run container
docker-compose -f docker/docker-compose.yml up -d

# Attach to container
docker exec -it bfmc_autonomous_vehicle bash

# Inside container
ros2 launch bfmc_nav navigation.launch.py
```

### Option 2: Docker CLI
```bash
# Build image
docker build -t bfmc_nav:latest -f docker/Dockerfile .

# Run container with GUI support
xhost +local:docker

docker run -it --rm \
  --name bfmc_nav \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/ws/src/bfmc_nav:rw \
  --device /dev/video0:/dev/video0 \
  bfmc_nav:latest
```

## Container Features

**Included Software:**
- ROS2 Humble
- OpenCV with Python bindings
- ros-humble-cv-bridge
- ros-humble-vision-opencv
- Camera drivers
- All Python dependencies from requirements.txt

**Volume Mounts:**
- Source code mounted for live development
- X11 socket for GUI applications

**Device Access:**
- Camera device (/dev/video0)
- Privileged mode for hardware access

## Building the Workspace

Inside the container:
```bash
# Navigate to workspace
cd /ws

# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Test
ros2 launch bfmc_nav simulation.launch.py
```

## Development Workflow
```bash
# 1. Start container
docker-compose -f docker/docker-compose.yml up -d

# 2. Enter container
docker exec -it bfmc_autonomous_vehicle bash

# 3. Make changes to code (outside or inside container)

# 4. Rebuild if needed
colcon build --packages-select bfmc_nav

# 5. Run
ros2 run bfmc_nav traffic_detector
```

## Troubleshooting

**GUI not working:**
```bash
# On host machine
xhost +local:docker

# Verify DISPLAY variable in container
echo $DISPLAY
```

**Camera not accessible:**
```bash
# Check device permissions
ls -l /dev/video0

# Add user to video group (on host)
sudo usermod -aG video $USER
```

**Build errors:**
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install
```

## Stopping Container
```bash
# Docker Compose
docker-compose -f docker/docker-compose.yml down

# Docker CLI
docker stop bfmc_nav
```

## Notes

- Container uses host network for ROS2 DDS communication
- Source code changes are reflected immediately (symlink install)
- Privileged mode required for camera and GPIO access
- X11 forwarding enables RViz and other GUI tools
```

**docker/.dockerignore**
```
# Build artifacts
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
*.egg-info/
dist/

# IDE
.vscode/
.idea/

# Git
.git/
.gitignore

# Rosbags
*.bag
*.db3

# Models
*.pth
*.pt
*.onnx

# Data
data/
rosbags/

# Documentation
docs/*.pdf
