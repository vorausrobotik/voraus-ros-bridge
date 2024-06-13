use std::sync::Arc;
use opcua::client::prelude::*;
use opcua::sync::*;


fn main() {
    // Optional - enable OPC UA logging
    opcua::console_logging::init();

    // Make the client configuration
    let mut client = ClientBuilder::new()
        .application_name("Simple Client")
        .application_uri("urn:SimpleClient")
        .product_uri("urn:SimpleClient")
        .trust_server_certs(true)
        .create_sample_keypair(true)
        .session_retry_limit(3)
        .client()
        .unwrap();

    // Create an endpoint. The EndpointDescription can be made from a tuple consisting of
    // the endpoint url, security policy, message security mode and user token policy.
    let endpoint: EndpointDescription = ("opc.tcp://localhost:48401/", "None", MessageSecurityMode::None, UserTokenPolicy::anonymous()).into();

    let session = client.connect_to_endpoint(endpoint, IdentityToken::Anonymous).unwrap();

    // Create a subscription and monitored items
    if subscribe_to_values(session.clone()).is_ok() {
        let _ = Session::run(session);
    } else {
        println!("Error creating subscription");
    }
}

fn subscribe_to_values(session: Arc<RwLock<Session>>) -> Result<(), StatusCode> {
    let session = session.write();
    // Create a subscription polling every 2s with a callback
    let subscription_id = session.create_subscription(2000.0, 10, 30, 0, 0, true, DataChangeCallback::new(|changed_monitored_items| {
        println!("Data change from server:");
        changed_monitored_items.iter().for_each(|item| print_value(item));
    }))?;
    // Create some monitored items
    let items_to_create: Vec<MonitoredItemCreateRequest> = ["100007", "100903", "100909"].iter()
        .map(|v| NodeId::new(1, *v).into()).collect();
    let _ = session.create_monitored_items(subscription_id, TimestampsToReturn::Both, &items_to_create)?;
    Ok(())
}

fn print_value(item: &MonitoredItem) {
   let node_id = &item.item_to_monitor().node_id;
   let data_value = item.last_value();
   if let Some(ref value) = data_value.value {
       println!("Item \"{}\", Value = {:?}", node_id, value);
   } else {
       println!("Item \"{}\", Value not found, error: {}", node_id, data_value.status.as_ref().unwrap());
   }
}