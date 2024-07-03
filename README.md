[![CI](https://github.com/vorausrobotik/voraus-ros-bridge/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/vorausrobotik/voraus-ros-bridge/actions/workflows/ci.yml)

# voraus-ros-bridge

Enables `voraus.core` integration within the ROS framework.

## Development

This repository provides a dev container to streamline the development process.
It offers an environment where all necessary development dependencies are installed and ready-to-use.
If you are using VSCode, the `Dev Containers` extension might be worth a shot. Other editors also have dev container
integrations (NeoVim, IntelliJ, ...).
However, it is still possible to use the dev container using plain docker commands.

### Build the container

`docker build -f .devcontainer/Dockerfile -t voraus-ros-bridge-dev .`

### Run the container

`docker run --rm -it --volume $(pwd):/home/developer/workspace voraus-ros-bridge-dev`

### Format the code

Run `cargo fmt`

### Analyze the code

Run `cargo clippy`

### Test the crate

Run `cargo test`

### Build the crates

Run `cargo build --release`

### Run the voraus_ros_bridge

Run `cargo run --release`

## via ROS:

run `cargo ament-build --install-base install/voraus_ros_bridge -- --release`.
Then `ros2 run voraus_ros_bridge voraus_ros_bridge`

### Custom message/service files

Create a separate ros message package just like you would in cpp.
Compile it.
Source the workspace of the message package. ($LD_LIBRARY_PATH has to be updated)
Include it in the cargo.toml and package.xml.
Build the consuming project with colcon the first time (in order to generate the cargo config.toml), after that cargo is fine.


### General Notes

ros2_rust only supports single threaded executors for now.
The ros bridge could benefit from both the multithreaded one and the static single threaded.
Might be a problem in the future but could potentially worked around by spawning multiple node.

## Ros msgs crates not published to creates.io
It would be nice to not have to manually invoke colcon in order to patch the dependencies etc.
Cargo provides a build.rs for such tasks, but since the dependency resolving happens before build.rs gets executed
it still fails if colcon build was not invoked manually before. This might be solvabe if the msgs crate where published 
normally on crates.io. Maybe we should restart the discussion on GitHub about this topic.