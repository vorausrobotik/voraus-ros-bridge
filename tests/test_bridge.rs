use std::{
    env,
    ffi::OsString,
    sync::{atomic::Ordering, mpsc, Arc},
    time::Duration,
};

use common::ManagedRosBridge;
use helpers::opc_ua_test_server::OPCUATestServer;

pub mod common;
pub mod helpers;

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
