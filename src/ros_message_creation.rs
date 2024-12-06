use std::{
    sync::{Arc, Mutex},
    time::{SystemTime, UNIX_EPOCH},
};

use builtin_interfaces::msg::Time;
use geometry_msgs::msg::{Pose, PoseStamped, Twist, TwistStamped, Vector3, Wrench, WrenchStamped};
use sensor_msgs::msg::JointState;
use std_msgs::msg::Header;

pub fn create_joint_state_msg(
    positions: &Arc<Mutex<Vec<f64>>>,
    velocities: &Arc<Mutex<Vec<f64>>>,
    efforts: &Arc<Mutex<Vec<f64>>>,
    frame_id: &str,
) -> JointState {
    JointState {
        header: Header {
            stamp: create_time_msg(),
            frame_id: frame_id.to_owned(),
        },
        name: vec![
            "joint1".to_string(),
            "joint2".to_string(),
            "joint3".to_string(),
            "joint4".to_string(),
            "joint5".to_string(),
            "joint6".to_string(),
        ],
        position: positions.lock().unwrap().to_vec(),
        velocity: velocities.lock().unwrap().to_vec(),
        effort: efforts.lock().unwrap().to_vec(),
    }
}

pub fn create_pose_stamped_msg(
    pose: &Arc<Mutex<Vec<f64>>>,
    quaternion: &Arc<Mutex<Vec<f64>>>,
    frame_id: &str,
) -> PoseStamped {
    let pose = pose.lock().unwrap().to_vec();
    let quaternion = quaternion.lock().unwrap().to_vec();

    PoseStamped {
        header: Header {
            stamp: create_time_msg(),
            frame_id: frame_id.to_owned(),
        },
        pose: Pose {
            position: geometry_msgs::msg::Point {
                x: pose[0],
                y: pose[1],
                z: pose[2],
            },
            orientation: geometry_msgs::msg::Quaternion {
                x: quaternion[1],
                y: quaternion[2],
                z: quaternion[3],
                w: quaternion[0],
            },
        },
    }
}

pub fn create_wrench_stamped_msg(tcp_force_torque: Vec<f64>, frame_id: &str) -> WrenchStamped {
    let force = Vector3 {
        x: tcp_force_torque[0],
        y: tcp_force_torque[1],
        z: tcp_force_torque[2],
    };
    let torque = Vector3 {
        x: tcp_force_torque[3],
        y: tcp_force_torque[4],
        z: tcp_force_torque[5],
    };
    WrenchStamped {
        header: Header {
            stamp: create_time_msg(),
            frame_id: frame_id.to_owned(),
        },
        wrench: Wrench { force, torque },
    }
}

pub fn create_twist_stamped_msg(tcp_velocities: Vec<f64>, frame_id: &str) -> TwistStamped {
    let linear = Vector3 {
        x: tcp_velocities[0],
        y: tcp_velocities[1],
        z: tcp_velocities[2],
    };
    let angular = Vector3 {
        x: tcp_velocities[3],
        y: tcp_velocities[4],
        z: tcp_velocities[5],
    };
    TwistStamped {
        header: Header {
            stamp: create_time_msg(),
            frame_id: frame_id.to_owned(),
        },
        twist: Twist { linear, angular },
    }
}

pub fn create_time_msg() -> Time {
    let system_timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    // Workaround for https://github.com/ros2-rust/ros2_rust/issues/385
    Time {
        sec: i32::try_from(system_timestamp.as_secs()).expect("This function will break in 2038."),
        nanosec: system_timestamp.subsec_nanos(),
    }
}

#[cfg(test)]
mod tests {

    use builtin_interfaces::msg::Time;
    use geometry_msgs::msg::{Point, Pose, Quaternion};
    use std_msgs::msg::Header;

    use super::*;
    #[test]
    fn test_create_pose_stamped_msg() {
        let pose = Arc::new(Mutex::new(vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0]));
        let quaternions = Arc::new(Mutex::new(vec![4.0, 5.0, 6.0, 7.0]));
        let expected_pose_stamped_msg = PoseStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 2 },
                frame_id: "base_link".to_string(),
            },
            pose: Pose {
                position: Point {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                orientation: Quaternion {
                    x: 5.0,
                    y: 6.0,
                    z: 7.0,
                    w: 4.0,
                },
            },
        };
        let mut pose_stamped_msg = create_pose_stamped_msg(&pose, &quaternions, "base_link");
        pose_stamped_msg.header.stamp.sec = 1;
        pose_stamped_msg.header.stamp.nanosec = 2;
        assert_eq!(expected_pose_stamped_msg, pose_stamped_msg);
    }
    #[test]
    fn test_create_time_msg_increases() {
        let first_msg = create_time_msg();
        let second_msg = create_time_msg();
        assert!(second_msg.nanosec > first_msg.nanosec);
    }
    #[test]
    fn test_create_joint_states_msg() {
        let positions = Arc::new(Mutex::new(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
        let velocities = Arc::new(Mutex::new(vec![7.0, 8.0, 9.0, 10.0, 11.0, 12.0]));
        let efforts = Arc::new(Mutex::new(vec![13.0, 14.0, 15.0, 16.0, 17.0, 18.0]));
        let expected_joint_states_msg = JointState {
            header: Header {
                stamp: Time { sec: 1, nanosec: 2 },
                frame_id: "base_link".to_string(),
            },
            name: vec![
                "joint1".to_string(),
                "joint2".to_string(),
                "joint3".to_string(),
                "joint4".to_string(),
                "joint5".to_string(),
                "joint6".to_string(),
            ],
            position: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            velocity: vec![7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
            effort: vec![13.0, 14.0, 15.0, 16.0, 17.0, 18.0],
        };
        let mut joint_states_msg =
            create_joint_state_msg(&positions, &velocities, &efforts, "base_link");
        joint_states_msg.header.stamp.sec = 1;
        joint_states_msg.header.stamp.nanosec = 2;
        assert_eq!(expected_joint_states_msg, joint_states_msg);
    }
    #[test]
    fn test_create_twist_stamped_msg() {
        let tcp_velocities = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let expected_twist_stamped_msg = TwistStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 2 },
                frame_id: "base_link".to_string(),
            },
            twist: Twist {
                linear: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                angular: Vector3 {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0,
                },
            },
        };
        let mut twist_stamped_msg = create_twist_stamped_msg(tcp_velocities, "base_link");
        twist_stamped_msg.header.stamp.sec = 1;
        twist_stamped_msg.header.stamp.nanosec = 2;
        assert_eq!(expected_twist_stamped_msg, twist_stamped_msg);
    }
    #[test]
    fn test_create_wrench_stamped_msg() {
        let tcp_force_torque = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let expected_wrench_stamped_msg = WrenchStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 2 },
                frame_id: "base_link".to_string(),
            },
            wrench: Wrench {
                force: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                torque: Vector3 {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0,
                },
            },
        };
        let mut wrench_stamped_msg = create_wrench_stamped_msg(tcp_force_torque, "base_link");
        wrench_stamped_msg.header.stamp.sec = 1;
        wrench_stamped_msg.header.stamp.nanosec = 2;
        assert_eq!(expected_wrench_stamped_msg, wrench_stamped_msg);
    }
}
