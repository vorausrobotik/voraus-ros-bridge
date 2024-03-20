ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

RUN apt-get update \
    && apt-get install -y \
    # install ros packages
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-lint \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
