ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Build and Install open62541
RUN git clone --recursive --depth 1 --branch v0.12.0 https://github.com/open62541pp/open62541pp.git && \
    cd open62541pp && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DUAPP_BUILD_EXAMPLES=OFF -DUAPP_BUILD_TESTS=OFF .. && \
    cmake --build . && \
    cmake --install . && \
    cd / && rm -r open62541pp

RUN apt-get update \
    && apt-get install -y \
    # install ros packages
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-lint \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
