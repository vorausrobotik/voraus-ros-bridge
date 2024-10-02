use std::sync::{Arc, Mutex};

use rclrs::Node;

use crate::{
    opc_ua_client::OPCUAClient,
    ros_publisher::JointStatesBuffer,
};

pub fn register_opcua_subscriptions(ros_node: Arc<Node>, opc_ua_client: Arc<Mutex<OPCUAClient>>) {
    let ros_node_copy_joint_states_buffer = Arc::clone(&ros_node);

    let joint_states_buffer = Arc::new(Mutex::new(JointStatesBuffer::new(
        ros_node_copy_joint_states_buffer,
    )));
    let joint_states_buffer_copy_position = Arc::clone(&joint_states_buffer);
    let joint_states_buffer_copy_velocity = Arc::clone(&joint_states_buffer);
    let joint_states_buffer_copy_effort = Arc::clone(&joint_states_buffer);

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100111,
            move |x| {
                joint_states_buffer_copy_position
                    .lock()
                    .unwrap()
                    .on_position_change(x);
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100115,
            move |x| {
                joint_states_buffer_copy_velocity
                    .lock()
                    .unwrap()
                    .on_velocity_change(x);
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100113,
            move |x| {
                joint_states_buffer_copy_effort
                    .lock()
                    .unwrap()
                    .on_effort_change(x);
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");
}
