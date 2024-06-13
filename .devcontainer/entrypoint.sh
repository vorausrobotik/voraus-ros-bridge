#!/bin/bash

# Build the package mounted in the container
echo "Building workspace with colcon"
. /opt/ros/humble/setup.sh && . /ros_deps/install/setup.sh && colcon build
echo "source /workspace/install/setup.sh" >> /root/.bashrc

# Run the CMD (either the default from the Dockerfile or the one provided as docker run argument)
exec "$@"
