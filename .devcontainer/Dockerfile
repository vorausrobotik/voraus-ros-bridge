ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install Rust and the cargo-ament-build plugin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.74.0 -y
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install cargo-ament-build

RUN pip install --upgrade pytest

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Install ros2_rust and its dependencies
RUN mkdir -p /ros_deps/src
WORKDIR /ros_deps
RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos
RUN echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc
RUN . /opt/ros/humble/setup.sh && colcon build
RUN echo "source /ros_deps/install/setup.sh" >> /root/.bashrc

RUN mkdir -p /workspace
WORKDIR /workspace

# Copy the generated cargo patches for the ros packages to the mounted workspace
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
