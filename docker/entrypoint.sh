#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace setup if it exists
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
