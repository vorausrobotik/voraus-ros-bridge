ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full
ENV SHELL /bin/bash

# Build and Install open62541
RUN git clone --depth 1 --branch v1.3.9 https://github.com/open62541/open62541.git && \
    cd open62541 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make && make install

RUN apt-get update \
    && apt-get install -y \
    # Install dependencies
    clangd \
    # Install ROS dependencies
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Install clangd-17
# RUN curl -s https://apt.llvm.org/llvm.sh | bash -s 17 && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Add new non root user
ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 

# Login as non root user
USER $USERNAME

# Enable ROS CLI
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc
