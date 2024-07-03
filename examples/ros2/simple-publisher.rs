use builtin_interfaces::msg::Time as TimeMsg;
use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::JointState as JointStateMsg;
use std::f64::consts::PI;
use std::{env, sync::Arc, thread, time::Duration, vec};
use std_msgs::msg::Header;
struct SimplePublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<JointStateMsg>>,
}

impl SimplePublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "joint_states").unwrap();
        let publisher = node
            .create_publisher("joint_states", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, publisher })
    }

    fn publish_data(&self, data: f64) -> Result<i32, RclrsError> {
        let time_stamp = self.node.get_clock().now().to_ros_msg().unwrap();
        let time_msgs = TimeMsg {
            sec: time_stamp.sec,
            nanosec: time_stamp.nanosec,
        };
        let msg: JointStateMsg = JointStateMsg {
            header: Header {
                stamp: time_msgs,
                frame_id: "1".to_string(),
            },
            name: vec!["Test Joint States".to_string()],
            position: vec![data, 3.0],
            velocity: vec![2.0, 3.3],
            effort: vec![],
        };
        self.publisher.publish(msg).unwrap();
        Ok(1)
    }
}

struct SineWave {
    amplitude: f64,
    frequency: f64,
    sample_rate: f64,
    current_sample: usize,
}

impl SineWave {
    fn new(amplitude: f64, frequency: f64, sample_rate: f64) -> Self {
        SineWave {
            amplitude,
            frequency,
            sample_rate,
            current_sample: 0,
        }
    }
}

impl Iterator for SineWave {
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        let t = self.current_sample as f64 / self.sample_rate;
        let value = self.amplitude * (2.0 * PI * self.frequency * t).sin();
        self.current_sample += 1;
        Some(value)
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut sine = SineWave::new(1.0, 10.0, 44100.0); // Amplitude 1.0, Frequency 440 Hz, Sample Rate 44100 Hz
    thread::spawn(move || loop {
        thread::sleep(Duration::from_nanos(100));

        publisher_other_thread
            .publish_data(sine.next().unwrap())
            .unwrap();
    });
    rclrs::spin(publisher.node.clone())
}
