use std::sync::Arc;

use opcua::client::prelude::{
    Client, ClientBuilder, DataChangeCallback, IdentityToken, MonitoredItem, MonitoredItemService,
    Session, SubscriptionService,
};
use opcua::crypto::SecurityPolicy;
use opcua::sync::RwLock;
use opcua::types::{
    MessageSecurityMode, MonitoredItemCreateRequest, NodeId, StatusCode, TimestampsToReturn,
    UserTokenPolicy,
};

const DEFAULT_URL: &str = "opc.tcp://127.0.0.1:4855";

pub fn launch_subscriber() -> Result<(), ()> {
    // Optional - enable OPC UA logging
    opcua::console_logging::init();

    let mut simple_subscriber = SimpleSubscriber::new(DEFAULT_URL);
    let _connection_result = simple_subscriber.connect();

    let guarded_session: Arc<RwLock<Session>> = simple_subscriber.session.unwrap();
    if let Err(result) = subscribe_to_variables(guarded_session.clone(), 2) {
        println!(
            "ERROR: Got an error while subscribing to variables - {}",
            result
        );
    } else {
        // Put SimpleSubscriber back together after partial borrow; hotfix until subscription is implemented.
        simple_subscriber.session=Some(guarded_session);
        // Loops forever. The publish thread will call the callback with changes on the variables
        simple_subscriber.run();
    }
    Ok(())
}

fn subscribe_to_variables(session: Arc<RwLock<Session>>, namespace: u16) -> Result<(), StatusCode> {
    let session = session.read();
    // Creates a subscription with a data change callback
    let publishing_interval_ms: f64 = 10.0;
    let lifetime_count: u32 = 10;
    let max_keep_alive_count: u32 = 30;
    let max_notifications_per_publish: u32 = 0;
    let priority: u8 = 0;
    let publishing_enabled: bool = true;

    let subscription_id = session.create_subscription(
        publishing_interval_ms,
        lifetime_count,
        max_keep_alive_count,
        max_notifications_per_publish,
        priority,
        publishing_enabled,
        DataChangeCallback::new(|changed_monitored_items| {
            println!("Data change from server:");
            changed_monitored_items
                .iter()
                .for_each(|item| print_value(item));
        }),
    )?;
    println!("Created a subscription with id = {}", subscription_id);

    // Create some monitored items
    let items_to_create: Vec<MonitoredItemCreateRequest> = ["ticks_since_launch"]
        .iter()
        .map(|v| NodeId::new(namespace, *v).into())
        .collect();
    let _ = session.create_monitored_items(
        subscription_id,
        TimestampsToReturn::Both,
        &items_to_create,
    )?;

    Ok(())
}

fn print_value(item: &MonitoredItem) {
    let node_id = &item.item_to_monitor().node_id;
    let data_value = item.last_value();
    if let Some(ref value) = data_value.value {
        println!("Item \"{}\", Value = {:?}", node_id, value);
    } else {
        println!(
            "Item \"{}\", Value not found, error: {}",
            node_id,
            data_value.status.as_ref().unwrap()
        );
    }
}

pub struct SimpleSubscriber {
    endpoint: String,
    session: Option<Arc<RwLock<Session>>>,
}

impl SimpleSubscriber {
    pub fn new<S: Into<String>>(endpoint: S) -> Self {
        Self {
            endpoint: endpoint.into(),
            session: None,
        }
    }

    pub fn connect(&mut self) -> Result<(), &str> {
        let mut client: Client = ClientBuilder::new()
            .application_name("Simple Subscriber")
            .application_uri("urn:SimpleSubscriber")
            .product_uri("urn:SimpleSubscriber")
            .trust_server_certs(true)
            .create_sample_keypair(true)
            .session_retry_limit(5)
            .client()
            .unwrap();
        match client.connect_to_endpoint(
            (
                self.endpoint.as_ref(),
                SecurityPolicy::None.to_str(),
                MessageSecurityMode::None,
                UserTokenPolicy::anonymous(),
            ),
            IdentityToken::Anonymous,
        ) {
            Ok(session) => {
                self.session = Some(session);
                Ok(())
            }
            Err(_) => Err("Could not connect to server."),
        }
    }

    pub fn run(self) {
        match self.session {
            Some(session) => Session::run(session),
            None => {
                eprintln!("Could not run inexistent session.");
            }
        }
    }
}
