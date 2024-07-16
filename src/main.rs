mod simple_opc_ua_subscriber;

use opcua::client::prelude::MonitoredItem;
use rclrs::{create_node, Context, RclrsError};
use ros_service_server::handle_service;
use simple_opc_ua_subscriber::SimpleSubscriber;
use std::{env, sync::Arc, thread, time::Duration};

mod ros_publisher;
mod ros_service_server;

use ros_publisher::{create_joint_state_msg, RosPublisher};

const FREQUENCY_HZ: u64 = 1000;

fn print_value(item: &MonitoredItem) {
    let node_id = &item.item_to_monitor().node_id;
    let data_value = item.last_value();
    if let Some(ref value) = data_value.value {
        println!("Item \"{}\", Value = {:?}", node_id, value);
    } else {
        println!(
            "Item \"{}\", Value not found, error: {}",
            node_id,
            data_value.status.as_ref().unwrap()
        );
    }
}

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

    opcua::console_logging::init();

    let mut simple_subscriber = SimpleSubscriber::new("opc.tcp://127.0.0.1:4855");
    let Ok(_connection_result) = simple_subscriber.connect() else {
        panic!("Connection could not be established, but is required.");
    };

    if let Err(result) =
        simple_subscriber.create_subscription(2, "ticks_since_launch", print_value, 10)
    {
        println!(
            "ERROR: Got an error while subscribing to variables - {}",
            result
        );
    };
    // Loops forever. The publish thread will call the callback with changes on the variables
    simple_subscriber.run();
    rclrs::spin(node_copy)
}
