use rclrs::{Node, RclrsError};
use std::{
    env,
    sync::{Arc, Mutex},
    time::Duration,
};

pub struct ServiceCaller {
    pub client: Arc<rclrs::Client<std_srvs::srv::Empty>>,
    pub node: Arc<Node>,
    pub request: std_srvs::srv::Empty_Request,
    pub number_of_calls: Arc<Mutex<u32>>,
}

impl ServiceCaller {
    pub fn new(service: &str) -> Result<Arc<Self>, RclrsError> {
        let context = rclrs::Context::new(env::args())?;

        let node = rclrs::create_node(&context, "minimal_client")?;

        let client = node.create_client::<std_srvs::srv::Empty>(service)?;

        let request = std_srvs::srv::Empty_Request {
            structure_needs_at_least_one_member: 0,
        };
        let action_caller = Arc::new(ServiceCaller {
            client,
            node,
            request,
            number_of_calls: Arc::new(Mutex::new(0)),
        });

        Ok(action_caller)
    }

    pub fn start(&self) {
        while !self.client.service_is_ready().unwrap() {
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }
    pub fn call(&self) {
        let num_calls_clone = Arc::clone(&self.number_of_calls);
        self.client
            .async_send_request_with_callback(
                &self.request,
                move |_response: std_srvs::srv::Empty_Response| {
                    let mut num_calls = num_calls_clone.lock().unwrap();
                    *num_calls += 1;
                },
            )
            .unwrap();

        let timeout = Some(Duration::new(5, 0));
        let node_clone = Arc::clone(&self.node);
        rclrs::spin_once(node_clone, timeout).unwrap();
    }
}
