use builtin_interfaces::msg::Time as TimeMsg;
use sensor_msgs::msg::JointState as JointStateMsg;
use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};
use std_msgs::msg::Header;

use rclrs::{Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use rosidl_runtime_rs::Message as RosMessage;

pub struct RosPublisher<T: RosMessage> {
    publisher: Arc<Publisher<T>>,
}

impl<T: RosMessage> RosPublisher<T> {
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError> {
        let publisher = node.create_publisher(topic, QOS_PROFILE_DEFAULT).unwrap();
        Ok(Self { publisher })
    }

    pub fn publish_data(&self, data: &T) -> Result<(), RclrsError> {
        self.publisher.publish(data).unwrap();
        Ok(())
    }
}

pub fn create_joint_state_msg(_node: &Node, data: f64) -> JointStateMsg {
    let system_timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    // Workaround for https://github.com/ros2-rust/ros2_rust/issues/385
    let time_msgs = TimeMsg {
        sec: i32::try_from(system_timestamp.as_secs()).expect("This function will break in 2038."),
        nanosec: system_timestamp.subsec_nanos(),
    };

    let joint_state_msg: JointStateMsg = JointStateMsg {
        header: Header {
            stamp: time_msgs,
            frame_id: "0".to_string(),
        },
        name: vec!["Test Joint States".to_string()],
        position: vec![data, 3.0],
        velocity: vec![2.0, 3.3],
        effort: vec![],
    };
    joint_state_msg
}
