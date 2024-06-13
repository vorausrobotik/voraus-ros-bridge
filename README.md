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

### Build the crate

Run `cargo build`
