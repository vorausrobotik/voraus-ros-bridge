mod simple_opc_ua_subscriber;

use opcua::types::Variant;
use rclrs::{create_node, Context, RclrsError};
use ros_service_server::handle_service;
use simple_opc_ua_subscriber::SimpleSubscriber;
use std::{env, sync::Arc};

mod ros_publisher;
mod ros_service_server;

use ros_publisher::{create_joint_state_msg, RosPublisher};

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let node = create_node(&context, "voraus_bridge_node")?;
    let node_copy = Arc::clone(&node);
    let joint_state_publisher = Arc::new(RosPublisher::new(&node, "joint_states").unwrap());

    let _server = node_copy
        .create_service::<voraus_interfaces::srv::Voraus, _>("add_two_ints", handle_service)?;

    opcua::console_logging::init();

    let mut simple_subscriber = SimpleSubscriber::new("opc.tcp://127.0.0.1:4855");
    let Ok(_connection_result) = simple_subscriber.connect() else {
        panic!("Connection could not be established, but is required.");
    };

    let callback = {
        let provider = Arc::clone(&joint_state_publisher);
        move |x: Variant| {
            println!("Value = {:?}", &x);
            let data_value: f64 = x
                .try_into()
                .expect("This should have been encapsulated f64 but wasn't.");
            let j_msg = create_joint_state_msg(data_value);
            provider
                .publish_data(&j_msg)
                .expect("Error while publishing.")
        }
    };
    simple_subscriber
        .create_subscription(2, "ticks_since_launch", callback, 10)
        .expect("ERROR: Got an error while subscribing to variables");
    // Loops forever. The publish thread will call the callback with changes on the variables
    simple_subscriber.run();
    rclrs::spin(node_copy)
}
