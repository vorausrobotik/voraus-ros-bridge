use std::{
    env,
    path::PathBuf,
    sync::{atomic::Ordering, Arc},
    time::Duration,
};

use common::wait_for_function_to_pass;
use subprocess::{Popen, PopenConfig, Redirection};

pub mod common;
pub mod helpers;

struct ManagedRosBridge {
    process: Popen,
}

impl ManagedRosBridge {
    fn new() -> subprocess::Result<Self> {
        // Start the ROS Brigde
        // We can't use ros2 run here because of https://github.com/ros2/ros2cli/issues/895
        let root_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");
        let mut path_to_executable = PathBuf::from(root_dir);
        path_to_executable
            .push("install/voraus-ros-bridge/lib/voraus-ros-bridge/voraus-ros-bridge");

        let process = Popen::create(
            &[path_to_executable],
            PopenConfig {
                stdout: Redirection::Pipe,
                stderr: Redirection::Pipe,
                detached: false,
                ..Default::default()
            },
        )?;

        Ok(ManagedRosBridge { process })
    }

    fn is_running(&mut self) -> bool {
        self.process.poll().is_none()
    }

    fn get_std_err(&mut self) -> Option<String> {
        let (_out, err) = self
            .process
            .communicate(None)
            .expect("Failed to capture output");
        err
    }

    fn terminate(&mut self) {
        let _ = self.process.terminate();
        let _ = self.process.wait();
    }
}

impl Drop for ManagedRosBridge {
    fn drop(&mut self) {
        if self.is_running() {
            self.terminate();
        }
    }
}

#[tokio::test]
async fn e2e_opc_ua_var_to_ros_topic() {
    // Start the OPC UA server in the background
    helpers::opc_ua_publisher_single_linear::run_rapid_clock().await;

    let mut _bridge_process = ManagedRosBridge::new().expect("Failed to start subprocess");

    // Spawn ROS Subscriber against a sample topic
    let subscription = Arc::new(
        helpers::ros_subscriber::Subscriber::new("joint_states_subscriber", "joint_states")
            .unwrap(),
    );
    let clone = Arc::clone(&subscription.node);
    // TODO: Figure out why this takes almost 10 s [RR-836]
    rclrs::spin_once(clone, Some(Duration::from_secs(25))).expect("Could not spin");

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

    rclrs::spin_once(subscription.node.clone(), Some(Duration::new(5, 0))).expect("Could not spin");

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
}

#[tokio::test]
async fn e2e_ros_service_to_opc_ua_call() {
    // Start the OPC UA server in the background
    helpers::opc_ua_publisher_single_linear::run_rapid_clock().await;

    let mut bridge_process = ManagedRosBridge::new().expect("Failed to start subprocess");

    let service_caller = Arc::new(
        helpers::ros_service_caller::ServiceCaller::new("enable_impedance_control").unwrap(),
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
