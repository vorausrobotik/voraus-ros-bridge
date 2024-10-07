use std::sync::{Arc, Mutex};

use geometry_msgs::msg::{TwistStamped, WrenchStamped};
use rclrs::Node;

use crate::{
    opc_ua_client::OPCUAClient,
    ros_message_creation::{create_twist_stamped_msg, create_wrench_stamped_msg},
    ros_publisher::{unpack_data, JointStatesBuffer, RosPublisher, TCPPoseBuffer},
};

pub fn register_opcua_subscriptions(ros_node: Arc<Node>, opc_ua_client: Arc<Mutex<OPCUAClient>>) {
    let ros_node_copy_joint_states_buffer = Arc::clone(&ros_node);
    let ros_node_copy_tcp_pose_buffer = Arc::clone(&ros_node);

    let joint_states_buffer = Arc::new(Mutex::new(JointStatesBuffer::new(
        ros_node_copy_joint_states_buffer,
    )));
    let joint_states_buffer_copy_position = Arc::clone(&joint_states_buffer);
    let joint_states_buffer_copy_velocity = Arc::clone(&joint_states_buffer);
    let joint_states_buffer_copy_effort = Arc::clone(&joint_states_buffer);

    let tcp_pose_buffer = Arc::new(Mutex::new(TCPPoseBuffer::new(
        ros_node_copy_tcp_pose_buffer,
    )));
    let tcp_pose_buffer_copy_pose = Arc::clone(&tcp_pose_buffer);
    let tcp_pose_buffer_copy_quaternion = Arc::clone(&tcp_pose_buffer);

    let tcp_twist_publisher: Arc<Mutex<RosPublisher<TwistStamped>>> = Arc::new(Mutex::new(
        RosPublisher::new(&ros_node, "~/tcp_twist").unwrap(),
    ));
    let tcp_wrench_publisher: Arc<Mutex<RosPublisher<WrenchStamped>>> = Arc::new(Mutex::new(
        RosPublisher::new(&ros_node, "~/tcp_wrench").unwrap(),
    ));

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

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100707,
            move |x| tcp_pose_buffer_copy_pose.lock().unwrap().on_pose_change(x),
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100710,
            move |x| {
                tcp_pose_buffer_copy_quaternion
                    .lock()
                    .unwrap()
                    .on_quaternion_change(x)
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100708,
            move |x| {
                tcp_twist_publisher
                    .lock()
                    .unwrap()
                    .publish_data(&create_twist_stamped_msg(unpack_data(x)))
                    .unwrap()
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");

    opc_ua_client
        .lock()
        .unwrap()
        .create_subscription(
            1,
            100711,
            move |x| {
                tcp_wrench_publisher
                    .lock()
                    .unwrap()
                    .publish_data(&create_wrench_stamped_msg(unpack_data(x)))
                    .unwrap()
            },
            0.0,
        )
        .expect("ERROR: Got an error while subscribing to variable");
}
