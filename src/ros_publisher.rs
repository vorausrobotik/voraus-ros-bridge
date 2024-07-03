use builtin_interfaces::msg::Time as TimeMsg;
use sensor_msgs::msg::JointState as JointStateMsg;
use std::sync::Arc;
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

pub fn create_joint_state_msg(node: &Node, data: f64) -> JointStateMsg {
    let time = node.get_clock().now().to_ros_msg().unwrap();
    // Workaround for https://github.com/ros2-rust/ros2_rust/issues/385
    let time_msgs = TimeMsg {
        sec: time.sec,
        nanosec: time.nanosec,
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
