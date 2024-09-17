mod simple_opc_ua_subscriber;

use opcua::types::Variant;
use rclrs::{create_node, Context, RclrsError};
use ros_service_server::handle_service;
use simple_opc_ua_subscriber::SimpleSubscriber;
use std::{env, sync::Arc};

mod ros_publisher;
mod ros_service_server;

use ros_publisher::{create_joint_state_msg, RosPublisher};

#[tokio::main]
async fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let node = create_node(&context, "voraus_bridge_node")?;
    let node_copy = Arc::clone(&node);
    let joint_state_publisher = Arc::new(RosPublisher::new(&node, "joint_states").unwrap());

    let _server = node_copy
        .create_service::<voraus_interfaces::srv::Voraus, _>("add_two_ints", handle_service)?;

    opcua::console_logging::init();

    let mut simple_subscriber = SimpleSubscriber::new("opc.tcp://127.0.0.1:4855");
    let Ok(_connection_result) = simple_subscriber.connect().await else {
        panic!("Connection could not be established, but is required.");
    };

    let callback = {
        let provider = Arc::clone(&joint_state_publisher);
        move |x: Variant| {
            println!("Value = {:?}", &x);
            let mut data_value: Vec<f64> = vec![];
            match x {
                Variant::Array(unwrapped) => {
                    unwrapped.values.into_iter().for_each(|value| {
                        let unpacked_value = value
                            .as_f64()
                            .expect("This should have been encapsulated f64 but wasn't.");
                        data_value.push(unpacked_value)
                    });
                }
                _ => panic!("Expected an array"),
            };
            let j_msg = create_joint_state_msg(data_value);
            provider
                .publish_data(&j_msg)
                .expect("Error while publishing.")
        }
    };
    let handle = simple_subscriber.run().await;
    let _ = simple_subscriber
        .create_subscription(1, "100111", callback, 10)
        .await;

    tokio::task::spawn(async move {
        println!("Spinning ROS");
        rclrs::spin(node_copy)
    });
    println!("Awaiting Handle");
    handle.await.unwrap();
    Ok(())
}
