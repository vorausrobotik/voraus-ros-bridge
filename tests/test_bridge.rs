use std::{
    env,
    ffi::OsString,
    iter::zip,
    sync::{atomic::Ordering, mpsc, Arc},
    time::Duration,
};

use common::{wait_for_function_to_pass, ManagedRosBridge};
use helpers::opc_ua_test_server::OPCUATestServer;

pub mod common;
pub mod helpers;

#[test]
fn test_joint_states() {
    let (assertion_tx, _assertion_rx) = mpsc::channel();
    let _server = OPCUATestServer::new(assertion_tx);

    let mut _bridge_process = ManagedRosBridge::new(None).expect("Failed to start subprocess");

    let subscription = Arc::new(
        helpers::ros_subscriber::Subscriber::new(
            "joint_states_subscriber",
            "/voraus_bridge_node/joint_states",
        )
        .unwrap(),
    );
    let clone = Arc::clone(&subscription.node);
    // TODO: Figure out why this takes almost 10 s [RR-836]
    rclrs::spin_once(clone, Some(Duration::from_secs(25))).expect("Could not spin");

    wait_for_function_to_pass(
        || {
            rclrs::spin_once(subscription.node.clone(), Some(Duration::new(5, 0)))
                .expect("Could not spin");
            let joint_state = subscription.data.lock().unwrap().clone().unwrap();
            let mut joint_state_flat = joint_state
                .position
                .into_iter()
                .chain(joint_state.velocity.into_iter().chain(joint_state.effort));
            joint_state_flat.all(|x| x != 0.0)
        },
        5000,
    )
    .unwrap();

    let number_of_messages_received = subscription.num_messages.load(Ordering::SeqCst);
    let first_value = subscription.data.lock().unwrap().clone().unwrap();
    assert!(number_of_messages_received >= 2);
    let joint_state_first_value_flat = first_value
        .position
        .into_iter()
        .chain(first_value.velocity.into_iter().chain(first_value.effort));

    wait_for_function_to_pass(
        || {
            rclrs::spin_once(subscription.node.clone(), Some(Duration::new(5, 0)))
                .expect("Could not spin");
            let joint_state = subscription.data.lock().unwrap().clone().unwrap();
            let joint_state_flat = joint_state
                .position
                .into_iter()
                .chain(joint_state.velocity.into_iter().chain(joint_state.effort));
            zip(joint_state_flat, joint_state_first_value_flat.clone())
                .all(|(current, first)| current > first)
        },
        5000,
    )
    .unwrap();
}

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

#[test]
fn e2e_frame_id_prefix() {
    let (assertion_tx, _assertion_rx) = mpsc::channel();
    let _server = OPCUATestServer::new(assertion_tx);

    let expected_frame_id_prefix = "robot42";
    let mut current_env: Vec<(OsString, OsString)> = env::vars_os().collect();
    current_env.push(("FRAME_ID_PREFIX".into(), expected_frame_id_prefix.into()));
    let mut _bridge_process =
        ManagedRosBridge::new(Some(current_env)).expect("Failed to start subprocess");

    // Spawn ROS Subscriber against a sample topic
    let subscription = Arc::new(
        helpers::ros_subscriber::Subscriber::new(
            "joint_states_subscriber",
            "/voraus_bridge_node/joint_states",
        )
        .unwrap(),
    );
    let clone = Arc::clone(&subscription.node);
    // TODO: Figure out why this takes almost 10 s [RR-836]
    rclrs::spin_once(clone, Some(Duration::from_secs(25))).expect("Could not spin");

    let number_of_messages_received = subscription.num_messages.load(Ordering::SeqCst);
    let received_frame_id = &subscription
        .data
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .header
        .frame_id
        .clone();
    assert_eq!(number_of_messages_received, 1);
    assert_eq!(
        *received_frame_id,
        format!("{}_base_link", expected_frame_id_prefix)
    );
}
