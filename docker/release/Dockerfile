ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Build and Install open62541
RUN git clone --depth 1 --branch v1.3.9 https://github.com/open62541/open62541.git && \
    cd open62541 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make && make install && \
    cd / && rm -r open62541

RUN apt-get update \
    && apt-get --no-install-recommends -y install \
    # install ros dependencies
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

COPY --chmod=755 ./docker/release/entrypoint.sh /
COPY ./ /workspace/src/

WORKDIR /workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \ 
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN rm -rf src build log

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-c", "ros2 launch voraus_ros_bringup voraus_ros_bridge.launch.py"]