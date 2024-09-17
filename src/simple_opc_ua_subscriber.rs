use opcua::client::{
    Client, ClientBuilder, DataChangeCallback, IdentityToken, Session,
    SessionEventLoop,
};
use opcua::crypto::SecurityPolicy;
use opcua::types::{
    MessageSecurityMode, MonitoredItemCreateRequest, NodeId,
    StatusCode, TimestampsToReturn, UserTokenPolicy, Variant,
};

use std::sync::Arc;
use std::time::Duration;
use tokio::task::JoinHandle;

pub struct SimpleSubscriber {
    endpoint: String,
    session: Option<Arc<Session>>,
    event_loop: Option<Arc<SessionEventLoop>>,
}

impl SimpleSubscriber {
    pub fn new<S: Into<String>>(endpoint: S) -> Self {
        Self {
            endpoint: endpoint.into(),
            session: None,
            event_loop: None,
        }
    }

    pub async fn connect(&mut self) -> Result<(), &str> {
        let mut client: Client = ClientBuilder::new()
            .application_name("Simple Subscriber")
            .application_uri("urn:SimpleSubscriber")
            .product_uri("urn:SimpleSubscriber")
            .trust_server_certs(true)
            .create_sample_keypair(true)
            .session_retry_limit(5)
            .client()
            .unwrap();

        let (session, event_loop) = client
            .new_session_from_endpoint(
                (
                    self.endpoint.as_ref(),
                    SecurityPolicy::None.to_str(),
                    MessageSecurityMode::None,
                    UserTokenPolicy::anonymous(),
                ),
                IdentityToken::Anonymous,
            )
            .await
            .unwrap();
        self.session = Some(Arc::clone(&session));
        self.event_loop = Some(Arc::new(event_loop));
        Ok(())
    }

    pub async fn run(&mut self) -> JoinHandle<StatusCode> {
        if let Some(event_loop_arc) = self.event_loop.take() {
            match Arc::try_unwrap(event_loop_arc) {
                Ok(event_loop) => {
                    let handle = event_loop.spawn();
                    self.session.as_ref().unwrap().wait_for_connection().await;
                    handle
                }
                Err(_) => {
                    panic!("Event loop is still shared, cannot take ownership.");
                }
            }
        } else {
            panic!("Event loop is not initialized.");
        }
    }

    pub async fn create_subscription<F>(
        &self,
        namespace: u16,
        node_id: &'static str,
        callback: F,
        period_ms: u64,
    ) -> Result<(), StatusCode>
    where
        F: Fn(Variant) + Send + Sync + 'static,
    {
        if self.session.is_none() {
            panic!("Not connected. Can't create subscriptions.");
        }
        println!(
            "Creating a subscription for ns={};{} to indirectly call the callback every {}ms.",
            namespace, node_id, period_ms
        );
        // Creates a subscription with a data change callback
        let publishing_interval_ms = Duration::from_millis(10);
        let lifetime_count: u32 = 10;
        let max_keep_alive_count: u32 = 30;
        let max_notifications_per_publish: u32 = 0;
        let priority: u8 = 0;
        let publishing_enabled: bool = true;

        let session = self.session.clone().unwrap();

        let subscription_id = session
            .create_subscription(
                publishing_interval_ms,
                lifetime_count,
                max_keep_alive_count,
                max_notifications_per_publish,
                priority,
                publishing_enabled,
                DataChangeCallback::new(move |data_value, _changed_monitored_items| {
                    println!("Data change from server|");
                    callback(data_value.value.unwrap());
                }),
            )
            .await
            .unwrap();
        println!("Created a subscription with id = {}", subscription_id);

        // Create some monitored items
        let items_to_create: Vec<MonitoredItemCreateRequest> = [node_id]
            .iter()
            .map(|v| NodeId::new(namespace, *v).into())
            .collect();

        let _ = session
            .create_monitored_items(subscription_id, TimestampsToReturn::Both, items_to_create)
            .await
            .unwrap();

        Ok(())
    }
}
