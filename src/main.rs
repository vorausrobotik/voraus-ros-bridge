mod opc_ua_client;

use env_logger::{Builder, Env};
use log::{debug, info};
use opc_ua_client::OPCUAClient;
use opcua::types::Variant;
use rclrs::{create_node, Context, RclrsError};
use ros_services::ROSServices;
use std::{
    env,
    sync::{Arc, Mutex},
};

mod ros_publisher;
mod ros_services;

use ros_publisher::{create_joint_state_msg, RosPublisher};

fn main() -> Result<(), RclrsError> {
    Builder::from_env(Env::default().default_filter_or("info"))
        .filter_module("opcua", log::LevelFilter::Warn)
        .init();
    let context = Context::new(env::args()).unwrap();
    let node = create_node(&context, "voraus_bridge_node")?;
    let node_copy = Arc::clone(&node);
    let joint_state_publisher = Arc::new(RosPublisher::new(&node, "joint_states").unwrap());

    let opc_ua_client = Arc::new(Mutex::new(OPCUAClient::new("opc.tcp://127.0.0.1:4855")));
    let Ok(_connection_result) = opc_ua_client.lock().unwrap().connect() else {
        panic!("Connection could not be established, but is required.");
    };

    let ros_services = Arc::new(ROSServices::new(Arc::clone(&opc_ua_client)));
    let _enable_impedance_control =
        node_copy.create_service::<std_srvs::srv::Empty, _>("enable_impedance_control", {
            let rsc = Arc::clone(&ros_services);
            move |request_header, request| rsc.enable_impedance_control(request_header, request)
        });

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
        .lock()
        .unwrap()
        .create_subscription(1, "100111", callback, 10)
        .expect("ERROR: Got an error while subscribing to variables");
    // Loops forever. The publish thread will call the callback with changes on the variables
    info!("Starting OPC UA client");
    let _session = opc_ua_client.lock().unwrap().run_async();
    info!("Spinning ROS");
    rclrs::spin(node_copy)
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_dummy() {
        assert_eq!(1, 1);
    }
}
