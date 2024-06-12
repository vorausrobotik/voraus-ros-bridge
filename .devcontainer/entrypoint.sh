#!/bin/bash

# Build the package mounted in the container
. /opt/ros/humble/setup.sh && . /ros_deps/install/setup.sh && colcon build
echo "source /workspace/install/setup.sh" >> /root/.bashrc
/bin/bash