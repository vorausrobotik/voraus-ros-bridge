use log::debug;
use opcua::types::Variant;
use sensor_msgs::msg::JointState as JointStateMsg;
use std::sync::{Arc, Mutex};

use crate::ros_message_creation::create_joint_state_msg;
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

pub struct JointStatesBuffer {
    current_positions: Arc<Mutex<Vec<f64>>>,
    current_velocities: Arc<Mutex<Vec<f64>>>,
    current_efforts: Arc<Mutex<Vec<f64>>>,
    publisher: Arc<Mutex<RosPublisher<JointStateMsg>>>,
}

impl JointStatesBuffer {
    pub fn new(ros_node: Arc<Node>) -> Self {
        Self {
            current_positions: Arc::new(Mutex::new(vec![0.0; 6])),
            current_velocities: Arc::new(Mutex::new(vec![0.0; 6])),
            current_efforts: Arc::new(Mutex::new(vec![0.0; 6])),
            publisher: Arc::new(Mutex::new(
                RosPublisher::new(&ros_node, "joint_states").unwrap(),
            )),
        }
    }

    fn publish_new_joint_state_message(&self, joint_state_message: JointStateMsg) {
        self.publisher
            .lock()
            .unwrap()
            .publish_data(&joint_state_message)
            .expect("Error while publishing.");
    }

    pub fn on_position_change(&mut self, input: Variant) {
        let joint_positions: Vec<f64> = unpack_data(input);
        *self.current_positions.lock().unwrap() = joint_positions;
        let joint_state_msg = create_joint_state_msg(
            &self.current_positions,
            &self.current_velocities,
            &self.current_efforts,
        );
        self.publish_new_joint_state_message(joint_state_msg);
    }

    pub fn on_velocity_change(&mut self, input: Variant) {
        let joint_velocities: Vec<f64> = unpack_data(input);
        *self.current_velocities.lock().unwrap() = joint_velocities;
        let joint_state_msg = create_joint_state_msg(
            &self.current_positions,
            &self.current_velocities,
            &self.current_efforts,
        );
        self.publish_new_joint_state_message(joint_state_msg);
    }

    pub fn on_effort_change(&mut self, input: Variant) {
        let joint_efforts: Vec<f64> = unpack_data(input);
        *self.current_efforts.lock().unwrap() = joint_efforts;
        let joint_state_msg = create_joint_state_msg(
            &self.current_positions,
            &self.current_velocities,
            &self.current_efforts,
        );
        self.publish_new_joint_state_message(joint_state_msg);
    }
}

pub fn unpack_data(x: Variant) -> Vec<f64> {
    debug!("Value = {:?}", &x);
    let mut data: Vec<f64> = vec![];
    match x {
        Variant::Array(unwrapped) => {
            unwrapped.values.into_iter().for_each(|value| {
                let unpacked_value = value
                    .as_f64()
                    .expect("This should have been encapsulated f64 but wasn't.");
                data.push(unpacked_value)
            });
        }
        _ => panic!("Expected an array"),
    }
    data
}
