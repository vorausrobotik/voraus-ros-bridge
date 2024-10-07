use std::sync::Arc;

use log::debug;
use opcua::client::prelude::{
    Client, ClientBuilder, DataChangeCallback, IdentityToken, MethodService, MonitoredItem,
    MonitoredItemService, Session, SessionCommand, SubscriptionService,
};
use opcua::crypto::SecurityPolicy;
use opcua::sync::RwLock;
use opcua::types::{
    CallMethodRequest, MessageSecurityMode, MonitoredItemCreateRequest, NodeId, StatusCode,
    TimestampsToReturn, UserTokenPolicy, Variant,
};
use tokio::sync::oneshot;

pub struct OPCUAClient {
    endpoint: String,
    session: Option<Arc<RwLock<Session>>>,
}

fn extract_value(item: &MonitoredItem) -> Variant {
    let data_value = item.last_value();
    data_value
        .value
        .clone()
        .expect("No value found - check value bit in EncodingMask.")
}

impl OPCUAClient {
    pub fn new<S: Into<String>>(endpoint: S) -> Self {
        Self {
            endpoint: endpoint.into(),
            session: None,
        }
    }

    pub fn connect(&mut self) -> Result<(), &str> {
        let mut client: Client = ClientBuilder::new()
            .application_name("voraus ROS brigde")
            .application_uri("urn:vorausRosBridge")
            .product_uri("urn:vorausRosBridge")
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

    pub fn create_subscription<F>(
        &self,
        namespace: u16,
        node_id: u32,
        callback: F,
        period_ms: f64,
    ) -> Result<(), StatusCode>
    where
        F: Fn(Variant) + Send + Sync + 'static,
    {
        if self.session.is_none() {
            panic!("Not connected. Can't create subscriptions.");
        }
        debug!(
            "Creating a subscription for ns={};{} to indirectly call the callback every {} ms.",
            namespace, node_id, period_ms
        );
        let cloned_session_lock = self.session.clone().unwrap();
        let session = cloned_session_lock.read();
        // Creates a subscription with a data change callback
        let lifetime_count: u32 = 10;
        let max_keep_alive_count: u32 = 30;
        let max_notifications_per_publish: u32 = 0;
        let priority: u8 = 0;
        let publishing_enabled: bool = true;

        let subscription_id = session.create_subscription(
            period_ms,
            lifetime_count,
            max_keep_alive_count,
            max_notifications_per_publish,
            priority,
            publishing_enabled,
            DataChangeCallback::new(move |changed_monitored_items| {
                debug!("Data change from server:");
                changed_monitored_items
                    .iter()
                    .for_each(|item| callback(extract_value(item)));
            }),
        )?;
        debug!("Created a subscription with id = {}", subscription_id);

        // Create some monitored items
        let items_to_create: Vec<MonitoredItemCreateRequest> = [node_id]
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

    pub fn call_method<T>(&self, object_id: NodeId, method_id: NodeId, args: Option<Vec<T>>)
    where
        T: Into<Variant>,
    {
        let cloned_session_lock = self.session.clone().unwrap();
        let session = cloned_session_lock.read();
        let arguments: Option<Vec<Variant>> =
            args.map(|vec| vec.into_iter().map(Into::into).collect());
        let method = CallMethodRequest {
            object_id,
            method_id,
            input_arguments: arguments,
        };
        let result = session.call(method).unwrap();
        debug!("result of call: {:?}", result);
    }

    pub fn run_async(&self) -> oneshot::Sender<SessionCommand> {
        let cloned_session_lock = self
            .session
            .clone()
            .expect("The session should be connected. Please call connect() first.");
        Session::run_async(cloned_session_lock)
    }
}
