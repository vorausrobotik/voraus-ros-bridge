# voraus-ros-bridge

Enables `voraus.core` integration within the ROS framework.

## Build the container

`docker build -f .devcontainer/Dockerfile -t voraus_ros_bridge .`

## Run the container

`docker run --rm -it --volume $(pwd):/workspace ros2_rust_dev /bin/bash`

Run this command in the root directory of this repository.
`docker compose run --rm voraus-ros-bridge`

## Build the crate

Navigate to the `src` directory within the dev container and run `cargo build`

## Test the crate

Navigate to the `src` directory within the dev container and run `cargo test`

## ROS Communication

### Topics

- Continuous Data Streams (e.g. sensor data, robot state)
- Publisher decides when data is sent
- Subscriber defines callbacks that are called once data is available

### Services

- Simple blocking call
- Remote procedure calls that terminate quickly

### Actions

- More complex non-blocking background processes
- Runs for a longer time but provides feedback during execution
- Can be preempted (which is implemented by the action server)

## OPC UA Communication

### Request - Response

### Pub Sub

### Data Access - Read/Write



Requirements

    - Publish all OPC UA attributes as topics
    - Map all OPC UA methods to ROS services (make sure that they are non blocking)


https://github.com/ros2-rust/ros2_rust
https://github.com/ros2-rust/ros2_rust/pull/390/files?short_path=c41224f#diff-c41224f73af17bdc406bc9d757c9978b02b28f29b1c6ab3a8590d32b7caff1a9
https://github.com/ros2-rust/ros2_rust/pull/295
https://github.com/sequenceplanner/r2r
https://github.com/locka99/opcua/blob/master/docs/setup.md
https://github.com/Mariunil/ros2-opcua
https://github.com/iirob/ros_opcua_communication/tree/kinetic-devel/ros_opcua_impl_python_opcua/scripts
https://gitlab.com/ros21923912/simple_ros2_node/-/tree/more_simple_nodes/src?ref_type=heads
