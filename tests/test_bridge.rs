use std::{
    process::Command,
    sync::{atomic::Ordering, Arc},
    time::Duration,
};

pub mod helpers;

#[tokio::test]
async fn e2e_opc_ua_var_to_ros_topic() {
    // Start the OPC UA server in the background
    helpers::opc_ua_publisher_single_linear::run_rapid_clock().await;

    println!("Starting ros bridge");
    let mut bridge = Command::new("ros2")
        .args(["run"])
        .args(["voraus_ros_bridge"])
        .args(["voraus_ros_bridge"])
        .spawn()
        .expect("Failed to run command");

    // Spawn ROS Subscriber against a sample topic
    println!("Creating Subscription");
    let subscription = Arc::new(
        helpers::ros_subscriber::Subscriber::new("joint_states_subscriber", "joint_states")
            .unwrap(),
    );
    let timeout = Some(Duration::new(5, 0));
    rclrs::spin_once(subscription.node.clone(), timeout).expect("Could not spin");

    let mut number_of_messages_received = subscription.num_messages.load(Ordering::SeqCst);
    let first_value = *subscription
        .data
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .position
        .first()
        .unwrap();
    assert_eq!(number_of_messages_received, 1);

    rclrs::spin_once(subscription.node.clone(), timeout).expect("Could not spin");

    number_of_messages_received = subscription.num_messages.load(Ordering::SeqCst);
    assert_eq!(number_of_messages_received, 2);
    let second_value = *subscription
        .data
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .position
        .first()
        .unwrap();

    println!("{}, {}", first_value, second_value);
    assert!(
        second_value > first_value,
        "{} is not greater than {}",
        second_value,
        first_value
    );
    bridge
        .kill()
        .expect("voraus-ros-bridge process could not be killed.");
    println!("done");
}
