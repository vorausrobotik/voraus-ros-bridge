# ros:humble
FROM ros@sha256:0dd6b5d4a9b113e543f8b3ea1d837e2bbf20e706398c3dd4bea5b245a4a8ec1e

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    htop \
    vim \
    python3-pip \
    python3-vcstool \
    bash-completion \
    sudo \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid "$USER_GID" "$USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" \
    # Add sudo support
    && echo "$USERNAME" ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/"$USERNAME" \
    && chmod 0440 /etc/sudoers.d/"$USERNAME"

USER $USERNAME

# Install Rust and the cargo-ament-build plugin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.74.0 -y
ENV PATH=/home/$USERNAME/.cargo/bin:$PATH
RUN cargo install cargo-ament-build
# Install autocompletion for rustup and cargo
RUN mkdir -p ~/.local/share/bash-completion/completions
RUN rustup completions bash >> ~/.local/share/bash-completion/completions/rustup
RUN rustup completions bash cargo >> ~/.local/share/bash-completion/completions/cargo

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Install ros2_rust and its dependencies
RUN mkdir -p /home/"$USERNAME"/ros_deps/src
WORKDIR /home/$USERNAME/ros_deps
RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos
RUN . /opt/ros/humble/setup.sh && colcon build

RUN echo "source /opt/ros/humble/setup.sh" >> /home/"$USERNAME"/.bashrc
RUN echo "source ~/ros_deps/install/setup.sh" >> /home/"$USERNAME"/.bashrc

ENV CARGO_TERM_COLOR=always
ENV SHELL /bin/bash

RUN mkdir -p /home/"$USERNAME"/workspace
WORKDIR /home/$USERNAME/workspace

COPY --chmod=777 .devcontainer/entrypoint /usr/local/bin/

ENTRYPOINT ["bash", "entrypoint"]
CMD ["/bin/bash"]
