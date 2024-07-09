mod simple_opc_ua_subscriber;

use rclrs::{create_node, Context, RclrsError};
use ros_service_server::handle_service;
use std::{env, sync::Arc, thread, time::Duration};

mod ros_publisher;
mod ros_service_server;

use ros_publisher::{create_joint_state_msg, RosPublisher};

const FREQUENCY_HZ: u64 = 1000;

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let node = create_node(&context, "voraus_bridge_node")?;
    let node_copy = node.clone();
    let joint_state_publisher = Arc::new(RosPublisher::new(&node, "joint_states").unwrap());
    let publisher_thread_throttle_us =
        ((1.0 / FREQUENCY_HZ as f64) * 1000.0 * 1000.0).round() as u64;
    let mut increment = 0.0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_micros(publisher_thread_throttle_us));
        let joint_state_msg = create_joint_state_msg(increment);
        joint_state_publisher
            .publish_data(&joint_state_msg)
            .unwrap();
        increment += 1.0;
    });
    println!("Starting to publish joint states with {} Hz", FREQUENCY_HZ);

    let _server = node_copy
        .create_service::<voraus_interfaces::srv::Voraus, _>("add_two_ints", handle_service)?;

    simple_opc_ua_subscriber::launch_subscriber().unwrap();
    rclrs::spin(node_copy)
}
