use rclrs::{create_node, Context, Node, RclrsError, Subscription, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::JointState as JointStateMsg;
use std::{
    env,
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
};

struct Subscriber {
    num_messages: AtomicU32,
    node: Arc<Node>,
    subscription: Mutex<Option<Arc<Subscription<JointStateMsg>>>>,
}

impl Subscriber {
    pub fn new(name: &str, topic: &str) -> Result<Arc<Self>, RclrsError> {
        let context = Context::new(env::args())?;
        let node = create_node(&context, name)?;
        let subscriber = Arc::new(Subscriber {
            num_messages: 0.into(),
            node,
            subscription: None.into(),
        });

        let minimal_subscriber_aux = Arc::clone(&subscriber);
        let subscription = subscriber.node.create_subscription::<JointStateMsg, _>(
            topic,
            QOS_PROFILE_DEFAULT,
            move |msg: JointStateMsg| {
                minimal_subscriber_aux.callback(msg);
            },
        )?;
        *subscriber.subscription.lock().unwrap() = Some(subscription);
        Ok(subscriber)
    }
    fn callback(&self, msg: JointStateMsg) {
        self.num_messages.fetch_add(1, Ordering::SeqCst);
        println!(
            "[{}] I heard: '{:?}' with timestamp {}.",
            self.node.name(),
            msg.position,
            msg.header.stamp.nanosec
        );
        println!(
            "[{}] (Got {} messages so far)",
            self.node.name(),
            self.num_messages.load(Ordering::SeqCst)
        );
    }
}

fn main() -> Result<(), RclrsError> {
    let subscription =
        Arc::new(Subscriber::new("joint_states_subscriber", "joint_states").unwrap());
    rclrs::spin(subscription.node.clone())
}
