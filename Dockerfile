ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Build and Install open62541
RUN git clone --depth 1 --branch v1.3.9 https://github.com/open62541/open62541.git && \
    cd open62541 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make && make install && \
    cd / && rm -r open62541

RUN apt-get update \
    && apt-get install -y \
    # install ros packages
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-lint \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
