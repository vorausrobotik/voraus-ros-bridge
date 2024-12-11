[![CI](https://github.com/vorausrobotik/voraus-ros-bridge/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/vorausrobotik/voraus-ros-bridge/actions/workflows/ci.yml)

# voraus-ros-bridge

Enables `voraus.core` integration within the ROS 2 framework.

## Quick Start

The easiest way to use the voraus ros bridge is to start it via the provided docker compose file.

`docker compose up`

Note that you have to run this command from within the directory where the `compose.yaml` is located.

Additionally, you need to log in into the GitHub container registry to gain access to the provided release image (see [GitHub Docs](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry) for instructions).

## State of Integration

Currently, the following voraus.core functionality is exposed to the ROS 2 ecosystem:

- Read the current joint states (on topic `joint_states`)
- Read the current tcp pose (on topic `tcp_pose`)
- Read the current tcp twist/velocities (on topic `tcp_twist`)
- Read the current tcp wrench/forces (on topic `tcp_wrench`)
- Set tcp wrench for impedance control (via topic `impedance_control/set_wrench`)
- Set stiffness for impedance control (via topic `impedance_control/set_stiffness`)
- Enable/disable impedance control mode (via service `impedance_control/enable` or `impedance_control/disable`)

## Development

This repository provides a dev container to streamline the development process (see https://containers.dev/ for details on dev containers).
It offers an environment where all necessary development dependencies are installed and ready-to-use.
If you are using VSCode, the `Dev Containers` extension might be worth a shot. Other editors also have dev container
integrations (NeoVim, IntelliJ, ...).
However, it is still possible to use the dev container using plain docker commands.

> [!CAUTION]
> IntelliJ's RustRover appears to have trouble opening terminals in this specific DevContainer in an even remotely timely manner.
> Launching a terminal might very well take 7 minutes, while demanding full load on one 2.3GHz core.
> For the time being it's advised to stick with plain docker commands or work in VSCode.

### Build the dev container

`docker build -f .devcontainer/Dockerfile -t voraus-ros-bridge-dev .`

### Run the dev container

`docker run --rm -it --volume $(pwd):/home/developer/workspace voraus-ros-bridge-dev`

### Format the code

Run `cargo fmt`

### Analyze the code

Run `cargo clippy --all-targets --all-features -- -Dwarnings`

### Run the unit tests

Run `cargo test --bins`

### Run the integration tests

Run `cargo test --test '*' -- --test-threads=1`

### Build the crate

Run `cargo build --release`

### Run the voraus-ros-bridge

Run `cargo run --release`
In order to get debug log output, run `RUST_LOG=DEBUG cargo run --release`

The bridge can also be started via ros run, therefore use the following instructions.

run `cargo ament-build --install-base install/voraus-ros-bridge -- --release`.
Then `ros2 run voraus-ros-bridge voraus-ros-bridge`

### Custom message/service files

Create a separate ROS message package just like you would in cpp.
Compile it.
Source the workspace of the message package. (`$LD_LIBRARY_PATH` has to be updated)
Include it in the `cargo.toml` and `package.xml`.
Build the consuming project with `colcon` the first time (in order to generate the `cargo` `config.toml`),
after that `cargo` is fine.


### General notes

`ros2_rust` only supports single threaded executors for now.
The ROS bridge could benefit from both the multithreaded and the static single threaded one.
Might be a problem in the future but could potentially worked around by spawning multiple nodes.

### ROS msgs crates not published to crates.io

It would be nice to not have to manually invoke `colcon` in order to patch the dependencies etc..
`cargo` provides a `build.rs` for such tasks, but since the dependency resolving happens before `build.rs` gets executed
it still fails if `colcon build` was not invoked manually before.
This might be solvable if the msgs crates where published normally on `crates.io`.
Maybe we should restart the discussion on GitHub about this topic.
https://github.com/ros2-rust/ros2_rust/issues/394

## License Notices

This project is licensed under the [MIT License](https://opensource.org/license/mit/).
