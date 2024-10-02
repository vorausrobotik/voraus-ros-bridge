mod opc_ua_client;

use env_logger::{Builder, Env};
use log::info;
use opc_ua_client::OPCUAClient;
use rclrs::{create_node, Context, RclrsError};
use register_subscriptions::register_opcua_subscriptions;
use ros_services::ROSServices;
use std::{
    env,
    sync::{Arc, Mutex},
};

mod register_subscriptions;
mod ros_message_creation;
mod ros_publisher;
mod ros_services;

fn main() -> Result<(), RclrsError> {
    Builder::from_env(Env::default().default_filter_or("info"))
        .filter_module("opcua", log::LevelFilter::Warn)
        .init();
    let context = Context::new(env::args()).unwrap();
    let ros_node = create_node(&context, "voraus_bridge_node")?;
    let ros_node_copy_spin = Arc::clone(&ros_node);
    let ros_node_copy_service = Arc::clone(&ros_node);

    let opc_ua_client = Arc::new(Mutex::new(OPCUAClient::new("opc.tcp://127.0.0.1:4855")));
    let Ok(_connection_result) = opc_ua_client.lock().unwrap().connect() else {
        panic!("Connection could not be established, but is required.");
    };
    let opc_ua_client_copy_run = Arc::clone(&opc_ua_client);
    let opc_ua_client_copy_services = Arc::clone(&opc_ua_client);

    register_opcua_subscriptions(ros_node, opc_ua_client);

    let ros_services = Arc::new(ROSServices::new(opc_ua_client_copy_services));
    let _enable_impedance_control = ros_node_copy_service
        .create_service::<std_srvs::srv::Empty, _>("enable_impedance_control", {
            let rsc = Arc::clone(&ros_services);
            move |request_header, request| rsc.enable_impedance_control(request_header, request)
        });

    info!("Starting OPC UA client");
    let _session = opc_ua_client_copy_run.lock().unwrap().run_async();
    info!("Spinning ROS");
    rclrs::spin(ros_node_copy_spin)
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_dummy() {
        assert_eq!(1, 1);
    }
}
