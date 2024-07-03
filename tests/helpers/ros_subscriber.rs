use rclrs::{create_node, Context, Node, RclrsError, Subscription, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::JointState as JointStateMsg;
use std::{
    env,
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
};

pub struct Subscriber {
    pub num_messages: AtomicU32,
    pub data: Arc<Mutex<Option<JointStateMsg>>>,
    pub node: Arc<Node>,
    subscription: Mutex<Option<Arc<Subscription<JointStateMsg>>>>,
}

impl Subscriber {
    pub fn new(name: &str, topic: &str) -> Result<Arc<Self>, RclrsError> {
        let context = Context::new(env::args())?;
        let node = create_node(&context, name)?;
        let subscriber = Arc::new(Subscriber {
            num_messages: 0.into(),
            data: Arc::new(Mutex::new(None)),
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
        println!("msg {}", msg.clone().position.first().unwrap());
        *self.data.lock().unwrap() = Some(msg);

        println!(
            "data {}",
            self.data
                .lock()
                .unwrap()
                .as_mut()
                .unwrap()
                .position
                .first()
                .unwrap()
        );
    }
}
