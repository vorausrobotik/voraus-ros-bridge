mod opc_ua_client;

use env_logger::{Builder, Env};
use geometry_msgs::msg::Wrench;
use log::info;
use opc_ua_client::OPCUAClient;
use opcua::types::NodeId;
use rclrs::{create_node, Context, RclrsError, Subscription, QOS_PROFILE_DEFAULT};
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
    let ros_node_copy_register = Arc::clone(&ros_node);
    let ros_node_copy_spin = Arc::clone(&ros_node);
    let ros_node_copy_service = Arc::clone(&ros_node);

    let env_var_name = "VORAUS_CORE_OPC_UA_ENDPOINT";
    let voraus_core_opc_ua_endpoint = match env::var(env_var_name) {
        Ok(val) => val,
        Err(_) => "opc.tcp://127.0.0.1:48401".to_string(),
    };
    let opc_ua_client = Arc::new(Mutex::new(OPCUAClient::new(voraus_core_opc_ua_endpoint)));
    let Ok(_connection_result) = opc_ua_client.lock().unwrap().connect() else {
        panic!("Connection could not be established, but is required.");
    };
    let opc_ua_client_copy = Arc::clone(&opc_ua_client);
    register_opcua_subscriptions(ros_node_copy_register, opc_ua_client_copy);

    let opc_ua_client_copy = Arc::clone(&opc_ua_client);
    let ros_services = Arc::new(ROSServices::new(opc_ua_client_copy));
    let _enable_impedance_control = ros_node_copy_service
        .create_service::<std_srvs::srv::Empty, _>("~/impedance_control/enable", {
            let rsc = Arc::clone(&ros_services);
            move |request_header, request| rsc.enable_impedance_control(request_header, request)
        });

    let _disable_impedance_control = ros_node_copy_service
        .create_service::<std_srvs::srv::Empty, _>("~/impedance_control/disable", {
            let rsc = Arc::clone(&ros_services);
            move |request_header, request| rsc.disable_impedance_control(request_header, request)
        });

    let opc_ua_client_copy = Arc::clone(&opc_ua_client);
    let _wrench_subscriber: Arc<Subscription<Wrench>> = ros_node.create_subscription(
        "~/impedance_control/set_wrench",
        QOS_PROFILE_DEFAULT,
        move |msg: Wrench| {
            opc_ua_client_copy.lock().unwrap().call_method(
                NodeId::new(1, 100182),
                NodeId::new(1, 100267),
                Some(vec![
                    msg.force.x,
                    msg.force.y,
                    msg.force.z,
                    msg.torque.x,
                    msg.torque.y,
                    msg.torque.z,
                ]),
            );
        },
    )?;
    let opc_ua_client_copy = Arc::clone(&opc_ua_client);
    let _stiffness_subscriber: Arc<Subscription<voraus_interfaces::msg::CartesianStiffness>> =
        ros_node.create_subscription(
            "~/impedance_control/set_stiffness",
            QOS_PROFILE_DEFAULT,
            move |msg: voraus_interfaces::msg::CartesianStiffness| {
                opc_ua_client_copy.lock().unwrap().call_method(
                    NodeId::new(1, 100182),
                    NodeId::new(1, 100265),
                    Some(vec![
                        msg.translation.x,
                        msg.translation.y,
                        msg.translation.z,
                        msg.rotation.x,
                        msg.rotation.y,
                        msg.rotation.z,
                    ]),
                );
            },
        )?;

    info!("Starting OPC UA client");
    let opc_ua_client_copy = Arc::clone(&opc_ua_client);
    let _session = opc_ua_client_copy.lock().unwrap().run_async();
    info!("Spinning ROS");
    rclrs::spin(ros_node_copy_spin)
}
