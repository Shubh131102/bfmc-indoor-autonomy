# Minimal template — adjust to your ROS2 distro
FROM ros:humble

# System deps
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*

# Python deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Workspace
WORKDIR /ws
COPY . /ws
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN colcon build
