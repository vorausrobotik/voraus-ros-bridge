ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full
ENV SHELL /bin/bash

# Build and Install open62541pp
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
    # Install dependencies
    clangd \
    # Install ROS dependencies
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
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
