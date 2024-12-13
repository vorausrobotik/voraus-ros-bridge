use std::sync::{mpsc, Arc};

use common::{wait_for_function_to_pass, ManagedRosBridge};
use helpers::opc_ua_test_server::OPCUATestServer;

pub mod common;
pub mod helpers;

#[test]
fn e2e_ros_service_to_opc_ua_call() {
    let (assertion_tx, _assertion_rx) = mpsc::channel();
    let _server = OPCUATestServer::new(assertion_tx);

    let mut bridge_process = ManagedRosBridge::new(None).expect("Failed to start subprocess");

    let service_caller = Arc::new(
        helpers::ros_service_caller::ServiceCaller::new(
            "/voraus_bridge_node/impedance_control/enable",
        )
        .unwrap(),
    );

    // TODO: Figure out why this takes almost 10 s [RR-836]
    service_caller.start();
    assert!(*service_caller.number_of_calls.lock().unwrap() == 0);
    service_caller.call();

    wait_for_function_to_pass(
        || *service_caller.number_of_calls.lock().unwrap() == 1,
        5000,
    )
    .unwrap();
    bridge_process.terminate();
    assert!(bridge_process
        .get_std_err()
        .unwrap()
        .contains("ROS service called"));
}
