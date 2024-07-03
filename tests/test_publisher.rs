use std::{
    process::Command,
    sync::{atomic::Ordering, Arc},
    time::Duration,
};

mod helpers;

#[test]
fn e2e_opc_ua_var_to_ros_topic() {
    println!("Starting ros bridge");
    let mut bridge = Command::new("ros2")
        .args(["run"])
        .args(["voraus_ros_bridge"])
        .args(["voraus_ros_bridge"])
        .spawn()
        .expect("Failed to run command");

    // Spawn OPC UA Server with a sample variable
    // Spawn ROS Subscriber against the sample topic
    println!("Creating Subscription");
    let subscription = Arc::new(
        helpers::ros_subscriber::Subscriber::new("joint_states_subscriber", "joint_states")
            .unwrap(),
    );
    let timeout = Some(Duration::new(5, 0));
    rclrs::spin_once(subscription.node.clone(), timeout).expect("Could not spin");

    let mut number_of_messages_received = subscription.num_messages.load(Ordering::SeqCst);
    let first_sin_value = *subscription
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
    let second_sin_value = *subscription
        .data
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .position
        .first()
        .unwrap();

    println!("{}, {}", first_sin_value, second_sin_value);
    assert!(
        second_sin_value > first_sin_value,
        "{} is not greater than {}",
        second_sin_value,
        first_sin_value
    );
    bridge
        .kill()
        .expect("voraus-ros-bridge process could not be killed.");
    println!("done");
}
