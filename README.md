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
- Move joints (via service `move_joints`)

## Configuration

The voraus ros bridge can be configured via environment variables.
If your are using the docker image, those environment variables are set in the compose.yml file.
The following setting can be made:

- VORAUS_CORE_OPC_UA_ENDPOINT (Mandatory): The OPC UA endpoint to reach the voraus.core motion server, defaults to `opc.tcp://127.0.0.1:48401`
- ROS_NAMESPACE (Optional): Can be used for wrapping the whole node into a namespace (e.g. `/robot1/voraus_ros_bridge/joint_states`) and allows distinction of topics, services, etc.
- FRAME_ID_PREFIX (Optional): Prefix for the frame ids to be able to distinguish between coordinate frames of different robots e.g. for visualization of multiple robots in rviz. For example for the `base_link` coordinate system and `FRAME_ID_PREFIX=robot1` the resulting frame id is now `robot1_base_link`

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

### How to add new voraus.core functionality to ROS

The voraus.core has an OPC UA interface which this bridge utilizes.
Hence, the first step to expose new voraus.core functionality to the ROS 2 world is to validate that the desired functionality is available within the OPC UA interface (right now this must be done by inspecting the OPC UA server via tools like [UaExpert](https://www.unified-automation.com/products/development-tools/uaexpert.html) or the [Python OPC UA client](https://github.com/FreeOpcUa/opcua-client-gui)).
Then, the OPC UA functionality must be mapped to an adequate ROS 2 equivalent.
For OPC UA variables most of the time this is a ROS topic but could theoretically also be a getter ROS service.
OPC UA methods should be mapped to ROS services but it could also be convenient to use ROS subscriptions (see the `impedance_control/set_wrench` topic).

To showcase the implementation of a new feature, the voraus.core `move_joints` functionality is used as an example in the following.
According to the voraus.core OPC UA specification, the `move_joints` method can be reached at `ns=1;i=100211` with the parent node at `ns=1;i=100210`.
The method has quiet a lot of arguments, so it's most convenient to call it via a ROS service.

A ROS service is specified by its input and output arguments separated by `---` in a .srv file.
Hence, create a new file called `MoveJoints.srv` within `voraus_interfaces/srv` containing the following:

```
bool relative
uint32 target_reference_cs
float64[] target_coordinate
uint32 arriving_cs
float64 velocity_scaling
bool with_blending
float64 blending_parameter
int32[] config_vector
uint32 command_id
bool manual_mode
---
```

After that, add the new file to the `CMakeLists.txt` of the `voraus_interfaces` package and compile it via `colcon build`.
As a result, you are now able to use the new service type within the rust project.

The next step is to add a new method to the `ROSServices` struct, which will be later used as a callback for the ROS service call.
This method defines the OPC UA object id and method id of the desired voraus.core function.
It is additionally responsible to call the OPC UA method with the correct arguments provided by the service request.

Next, create a new ROS service in the `main.rs` and register the previously created method as a callback.

The new method can be added in the `test_bridge_methods` module in order to assert that it is properly propagated.
Run the integration test to verify the new test gets executed and passes.
Lastly, verify that it actually works E2E by starting a voraus.core and test the new functionality.

> **Note**  
Don't forget to add the new functionality to the README's State of Integration section.

For the concrete implementation of this example see this [pull request](https://github.com/vorausrobotik/voraus-ros-bridge/pull/74) (Note that the implementation details will likely change over time).

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

The licenses of dependencies can be found in the [LICENSES](./LICENSES) folder.
