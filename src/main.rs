mod opc_ua_client;

use env_logger::{Builder, Env};
use log::debug;
use opc_ua_client::OPCUAClient;
use opcua::types::Variant;
use rclrs::{create_node, Context, RclrsError};
use ros_service_server::handle_service;
use std::{env, sync::Arc};

mod ros_publisher;
mod ros_service_server;

use ros_publisher::{create_joint_state_msg, RosPublisher};

fn main() -> Result<(), RclrsError> {
    Builder::from_env(Env::default().default_filter_or("info"))
        .filter_module("opcua", log::LevelFilter::Warn)
        .init();
    let context = Context::new(env::args()).unwrap();
    let node = create_node(&context, "voraus_bridge_node")?;
    let node_copy = Arc::clone(&node);
    let joint_state_publisher = Arc::new(RosPublisher::new(&node, "joint_states").unwrap());

    let _server = node_copy
        .create_service::<voraus_interfaces::srv::Voraus, _>("add_two_ints", handle_service)?;

    let mut opc_ua_client = OPCUAClient::new("opc.tcp://127.0.0.1:4855");
    let Ok(_connection_result) = opc_ua_client.connect() else {
        panic!("Connection could not be established, but is required.");
    };

    let callback = {
        let provider = Arc::clone(&joint_state_publisher);
        move |x: Variant| {
            debug!("Value = {:?}", &x);
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
    opc_ua_client
        .create_subscription(1, "100111", callback, 10)
        .expect("ERROR: Got an error while subscribing to variables");
    // Loops forever. The publish thread will call the callback with changes on the variables
    opc_ua_client.run();
    rclrs::spin(node_copy)
}
